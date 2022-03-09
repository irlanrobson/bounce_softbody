/*
* Copyright (c) 2016-2019 Irlan Robson
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <bounce_softbody/dynamics/forces/tetrahedron_element_force.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/sparse_mat33.h>

// This work is based on the paper "Interactive Virtual Materials" written by 
// Matthias Mueller Fischer
// The paper is available here:
// http://matthias-mueller-fischer.ch/publications/GI2004.pdf

// However, in the paper the force Jacobians are approximate to ensure A is PD.
// Therefore, this may lead to instabilities for large deformations.
// Basically, increasing the stiffness damping or taking smaller time steps may 
// help preventing those unstable configurations.

// Compute the elasticity matrix given Young modulus and Poisson's ratio
// This is a 6 x 6 matrix 
static B3_FORCE_INLINE void b3ComputeD(scalar out[36],
	scalar E, scalar nu)
{
	scalar lambda = (nu * E) / ((scalar(1) + nu) * (scalar(1) - scalar(2) * nu));
	scalar mu = E / (scalar(2) * (scalar(1) + nu));

	scalar D[36] =
	{
		lambda + 2 * mu, lambda,          lambda,          0,  0,  0,
		lambda,          lambda + 2 * mu, lambda,          0,  0,  0,
		lambda,          lambda,          lambda + 2 * mu, 0,  0,  0,
		0, 				 0, 			  0, 			   mu, 0,  0,
		0, 				 0, 			  0, 			   0,  mu, 0,
		0, 			 	 0, 			  0, 			   0,  0,  mu
	};

	for (uint32 i = 0; i < 36; ++i)
	{
		out[i] = D[i];
	}
}

// Compute the strain-displacement matrix.
// This is a 6 x 12 matrix.
// 
// B = [dN1/dx 	0 		0 		dN2/dx 		0 		0 		dN3/dx 	0 		0 		dN4/dx 		0      	0	  ]
//	   [0      	dN1/dy 	0 		0      		dN2/dy 	0     	0 		dN3/dy  0   	0      		dN4/dy 	0	  ]
//     [0 		0 		dN1/dz 	0 			0 		dN2/dz 	0 		0 		dN3/dz 	0 			0 		dN4/dz]
//     [dN1/dy 	dN1/dx 	0 		dN2/dy 		dN2/dx 	0 		dN3/dy 	dN3/dx 	0 		dN4/dy 		dN4/dx 	0	  ]
//     [0 		dN1/dz 	dN1/dy 	0 			dN2/dz 	dN2/dy 	0 		dN3/dz 	dN3/dy 	0 			dN4/dz 	dN4/dy]
//     [dN1/dz 	0 		dN1/dx 	dN2/dz 		0 		dN2/dx 	dN3/dz 	0 		dN3/dx 	dN4/dz 		0 		dN4/dx]
//
// The shape functions are the Barycentric coordinates of a point to a tetrahedron:
//
// N1 * p1 + N2 * p2 + N3 * p3 + N4 * p4 = p
//
// N1 + N2 + N3 + N4 = 1
//
// Subtract p1 from both sides:
// 
// N2 * (p2 - p1) + N3 * (p3 - p1) + N4 * (p4 - p1) = p - p1
// 
// [p2 - p1 p3 - p1 p4 - p1][N2] = p - p1
//                          [N3]
//                          [N4]
// 
// Solve for N:
// 
// [N2] = E^-1 * [p - p1]
// [N3]
// [N4]
// 
// N1 = 1 - N2 - N3 - N4
// 
// E = [p2 - p1 p3 - p1 p4 - p1]
// 
// E^-1 = [exx eyx ezx]
//		  [exy eyy ezy]
//	      [exz eyz ezz]
// 
// Therefore,
// 
// [N2] = [exx * (px - p1x) + eyx * (py - p1y) + ezx * (pz - p1z)] 
// [N3]   [exy * (px - p1x) + eyy * (py - p1y) + ezy * (pz - p1z)]
// [N4]   [exz * (px - p1x) + eyz * (py - p1y) + ezz * (pz - p1z)]
// 
// N1 = 1 - N2 - N3 - N4
// 
// Differentiate:
//
// dN2/dx = exx
// dN3/dx = exy
// dN4/dx = exz
// dN1/dx = - dN2/dx - dN3/dx - dN4/dx
// 
// dN2/dy = eyx
// dN3/dy = eyy
// dN4/dy = eyz
// dN1/dy = - dN2/dy - dN3/dy - dN4/dy
// 
// dN2/dz = ezx
// dN3/dz = ezy 
// dN4/dz = ezz
// dN1/dz = - dN2/dz - dN3/dz - dN4/dz
static B3_FORCE_INLINE void b3ComputeB(scalar out[72],
	const b3Mat33& invE)
{
	scalar dN2dx = invE.x.x;
	scalar dN3dx = invE.x.y;
	scalar dN4dx = invE.x.z;
	scalar dN1dx = -dN2dx - dN3dx - dN4dx;

	scalar dN2dy = invE.y.x;
	scalar dN3dy = invE.y.y;
	scalar dN4dy = invE.y.z;
	scalar dN1dy = -dN2dy - dN3dy - dN4dy;

	scalar dN2dz = invE.z.x;
	scalar dN3dz = invE.z.y;
	scalar dN4dz = invE.z.z;
	scalar dN1dz = -dN2dz - dN3dz - dN4dz;

	scalar B[72] =
	{
		dN1dx, 0, 0, dN1dy, 0, dN1dz,
		0, dN1dy, 0, dN1dx, dN1dz, 0,
		0, 0, dN1dz, 0, dN1dy, dN1dx,
		dN2dx, 0, 0, dN2dy, 0, dN2dz,
		0, dN2dy, 0, dN2dx, dN2dz, 0,
		0, 0, dN2dz, 0, dN2dy, dN2dx,
		dN3dx, 0, 0, dN3dy, 0, dN3dz,
		0, dN3dy, 0, dN3dx, dN3dz, 0,
		0, 0, dN3dz, 0, dN3dy, dN3dx,
		dN4dx, 0, 0, dN4dy, 0, dN4dz,
		0, dN4dy, 0, dN4dx, dN4dz, 0,
		0, 0, dN4dz, 0, dN4dy, dN4dx
	};

	for (uint32 i = 0; i < 72; ++i)
	{
		out[i] = B[i];
	}
}

// Return the element in a block matrix given the indices 
// of the element in its corresponding expanded matrix.
static B3_FORCE_INLINE scalar& b3GetElement(b3Mat33 K[16], uint32 i, uint32 j)
{
	B3_ASSERT(i < 3 * 4);
	B3_ASSERT(j < 3 * 4);

	uint32 i0 = i / 3;
	uint32 j0 = j / 3;

	b3Mat33& a = K[i0 + 4 * j0];

	uint32 ii = i - 3 * i0;
	uint32 jj = j - 3 * j0;

	return a(ii, jj);
}

// Convert a 12-by-12 matrix to its 3-by-3 block form.
static B3_FORCE_INLINE void b3SetK(b3Mat33 K[16], scalar Ke[144])
{
	for (uint32 i = 0; i < 12; ++i)
	{
		for (uint32 j = 0; j < 12; ++j)
		{
			scalar k1 = Ke[i + 12 * j];
			scalar& k2 = b3GetElement(K, i, j);

			k2 = k1;
		}
	}
}

b3TetrahedronElementForce::b3TetrahedronElementForce(const b3TetrahedronElementForceDef* def)
{
	m_type = e_tetrahedronElementForce;
	m_meshIndex = def->meshIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_p4 = def->p4;
	m_x1 = def->v1;
	m_x2 = def->v2;
	m_x3 = def->v3;
	m_x4 = def->v4;
	m_E = def->youngModulus;
	m_nu = def->poissonRatio;
	m_stiffnessDamping = def->stiffnessDamping;
	m_q.SetIdentity();

	ResetElementData();
}

bool b3TetrahedronElementForce::HasParticle(const b3Particle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle || m_p4 == particle;
}

void b3TetrahedronElementForce::ResetElementData()
{
	b3Vec3 x1 = m_x1, x2 = m_x2;
	b3Vec3 x3 = m_x3, x4 = m_x4;

	b3Vec3 e1 = x2 - x1;
	b3Vec3 e2 = x3 - x1;
	b3Vec3 e3 = x4 - x1;

	b3Mat33 E(e1, e2, e3);
	
	scalar det = b3Det(e1, e2, e3);
	
	// Volume
	scalar V = b3Abs(det) / scalar(6);
	
	B3_ASSERT(det != scalar(0));
	det = scalar(1) / det;
	m_invE = det * b3Adjugate(E);

	// 6 x 6
	scalar D[36];
	b3ComputeD(D, m_E, m_nu);

	// 6 x 12
	scalar B[72];
	b3ComputeB(B, m_invE);

	// 12 x 6
	scalar BT[72];
	b3Transpose(BT, B, 6, 12);

	// 12 x 6
	scalar BT_D[72];
	b3Mul(BT_D, BT, 12, 6, D, 6, 6);

	// 12 x 12
	scalar BT_D_B[144];
	b3Mul(BT_D_B, BT_D, 12, 6, B, 6, 12);
	for (uint32 i = 0; i < 144; ++i)
	{
		BT_D_B[i] *= V;
	}

	// Convert K to block form.
	b3SetK(m_K, BT_D_B);
}

void b3TetrahedronElementForce::ClearForces()
{
}

// Extract rotation from deformation
// "A Robust Method to Extract the Rotational Part of Deformations", Matthias Muller et al.
static b3Quat b3ExtractRotation(const b3Mat33& A, const b3Quat& q0, uint32 maxIterations = 32)
{
	b3Quat q = q0;

	for (uint32 iteration = 0; iteration < maxIterations; ++iteration)
	{
		b3Mat33 R = q.GetRotationMatrix();

		scalar s = b3Abs(b3Dot(R.x, A.x) + b3Dot(R.y, A.y) + b3Dot(R.z, A.z));

		if (s == scalar(0))
		{
			break;
		}

		const scalar kTol = scalar(1.0e-9);

		scalar inv_s = scalar(1) / s + kTol;

		b3Vec3 v = b3Cross(R.x, A.x) + b3Cross(R.y, A.y) + b3Cross(R.z, A.z);

		b3Vec3 omega = inv_s * v;

		scalar w = b3Length(omega);

		if (w < kTol)
		{
			break;
		}

		b3Quat omega_q;
		omega_q.SetAxisAngle(omega / w, w);

		q = omega_q * q;
		q.Normalize();
	}

	return q;
}

void b3TetrahedronElementForce::ComputeForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& x = *data->x;
	const b3DenseVec3& v = *data->v;
	
	b3DenseVec3& f = *data->f;
	
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	uint32 i1 = m_p1->m_solverId;
	uint32 i2 = m_p2->m_solverId;
	uint32 i3 = m_p3->m_solverId;
	uint32 i4 = m_p4->m_solverId;

	b3Vec3 x1 = m_x1;
	b3Vec3 x2 = m_x2;
	b3Vec3 x3 = m_x3;
	b3Vec3 x4 = m_x4;

	b3Vec3 p1 = x[i1];
	b3Vec3 p2 = x[i2];
	b3Vec3 p3 = x[i3];
	b3Vec3 p4 = x[i4];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];
	b3Vec3 v4 = v[i4];

	b3Vec3 e1 = p2 - p1;
	b3Vec3 e2 = p3 - p1;
	b3Vec3 e3 = p4 - p1;

	// Deformation
	b3Mat33 E(e1, e2, e3);

	// Deformation gradient
	b3Mat33 F = E * m_invE;

	// Extract rotation from deformation gradient
	b3Quat q = b3ExtractRotation(F, m_q);
	
	m_q = q;

	// Rotation matrix
	b3Mat33 R = q.GetRotationMatrix();

	// Inverse rotation
	b3Mat33 RT = b3Transpose(R);

	// Element stiffness matrix
	b3Mat33 K[16];
	
	// K = R * K0 * R^T
	for (uint32 i = 0; i < 4; ++i)
	{
		for (uint32 j = 0; j < 4; ++j)
		{
			b3Mat33 k = R * m_K[i + 4 * j] * RT;
			
			K[i + 4 * j] = k;
		}
	}

	uint32 is[4] = { i1, i2, i3, i4 };
	
	for (uint32 i = 0; i < 4; ++i)
	{
		uint32 vi = is[i];

		for (uint32 j = 0; j < 4; ++j)
		{
			uint32 vj = is[j];

			b3Mat33 k = K[i + 4 * j];

			// Negate K
			dfdx(vi, vj) -= k;
		}
	}

	// Displacements in unrotated frame
	b3Vec3 us[4];
	us[0] = RT * p1 - x1;
	us[1] = RT * p2 - x2;
	us[2] = RT * p3 - x3;
	us[3] = RT * p4 - x4;

	// Forces in unrotated frame
	b3Vec3 fs[4];
	for (uint32 i = 0; i < 4; ++i)
	{
		fs[i].SetZero();
		for (uint32 j = 0; j < 4; ++j)
		{
			fs[i] += m_K[i + 4 * j] * us[j];
		}
	}

	// Rotate the forces to deformed frame
	fs[0] = R * fs[0];
	fs[1] = R * fs[1];
	fs[2] = R * fs[2];
	fs[3] = R * fs[3];

	// Negate f
	f[i1] -= fs[0];
	f[i2] -= fs[1];
	f[i3] -= fs[2];
	f[i4] -= fs[3];

	if (m_stiffnessDamping > scalar(0))
	{
		b3Vec3 vs[4] = { v1, v2, v3, v4 };

		// Lagged Rayleigh damping force
		// fd ~= -k * K(x) * v
		b3Vec3 fds[4];
		for (uint32 i = 0; i < 4; ++i)
		{
			fds[i].SetZero();
			for (uint32 j = 0; j < 4; ++j)
			{
				b3Mat33 k = K[i + 4 * j];		
				
				fds[i] += m_stiffnessDamping * k * vs[j];
			}
		}

		// Negate f
		f[i1] -= fds[0];
		f[i2] -= fds[1];
		f[i3] -= fds[2];
		f[i4] -= fds[3];

		for (uint32 i = 0; i < 4; ++i)
		{
			uint32 vi = is[i];

			for (uint32 j = 0; j < 4; ++j)
			{
				uint32 vj = is[j];

				b3Mat33 k = K[i + 4 * j];

				// Negate K
				dfdv(vi, vj) -= m_stiffnessDamping * k;
			}
		}
	}
}
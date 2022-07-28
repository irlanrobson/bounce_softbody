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

#include <bounce_softbody/dynamics/forces/triangle_element_force.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/sparse_mat33.h>

// Implementation of "Adaptive cloth simulation using corotational finite elements" by 
// Jan Bender and Crispin Deul.

// Compute the orthotropic elastic tensor given Young Modulus and Poisson's Ratio.
// This is a 3x3 matrix.
static B3_FORCE_INLINE b3Mat33 b3ComputeC(scalar Ex, scalar Ey, scalar Es,
	scalar nu_xy, scalar nu_yx)
{
	scalar s = scalar(1) - nu_xy * nu_yx;
	B3_ASSERT(s != scalar(0));

	b3Mat33 C;
	C.x.x = Ex / s;
	C.x.y = Ey * nu_xy / s;
	C.x.z = scalar(0);

	C.y.x = Ex * nu_yx / s;
	C.y.y = Ey / s;
	C.y.z = scalar(0);

	C.z.x = scalar(0);
	C.z.y = scalar(0);
	C.z.z = Es;

	return C;
}

// Compute the strain-displacement matrix.
// This is a 3x6 matrix.
// 
// B = [dN1/dx	0		dN2/dx	0		dN3/dx	0     ]
//     [0       dN1/dy  0       dN2/dy  0       dN3/dy] 
//     [dN1/dy  dN1/dx  dN2/dy  dN2/dx  dN3/dy  dN3/dx]
//
// The shape functions are the Barycentric coordinates of a point to a triangle:
//
// N1 * p1 + N2 * p2 + N3 * p3 = p
//
// N1 + N2 + N3 = 1
//
// Subtract p1 from both sides:
// 
// N2 * (p2 - p1) + N3 * (p3 - p1) = p - p1
// 
// [p2 - p1 p3 - p1][N2] = p - p1
//                  [N3]
// 
// Solve for N:
// 
// [N2] = S^-1 * [p - p1]
// [N3]
// 
// N1 = 1 - N2 - N3
// 
// S = [p2 - p1 p3 - p1]
// 
// S^-1 = [sxx syx]
//		  [sxy syy]
// 
// Therefore,
// 
// [N2] = [sxx * (px - p1x) + syx * (py - p1y)] 
// [N3]   [sxy * (px - p1x) + syy * (py - p1y)]
// 
// N1 = 1 - N2 - N3
// 
// Differentiate:
//
// dN2/dx = sxx
// dN3/dx = sxy
// dN1/dx = - dN2/dx - dN3/dx
// 
// dN2/dy = syx
// dN3/dy = syy
// dN1/dy = - dN2/dy - dN3/dy
static B3_FORCE_INLINE void b3ComputeB(scalar out[18],
	const b3Mat22& invS)
{
	scalar dN2dx = invS.x.x;
	scalar dN3dx = invS.x.y;
	scalar dN1dx = -dN2dx - dN3dx;

	scalar dN2dy = invS.y.x;
	scalar dN3dy = invS.y.y;
	scalar dN1dy = -dN2dy - dN3dy;

	scalar B[18]
	{
		dN1dx, 0, dN1dy,
		0, dN1dy, dN1dx,

		dN2dx, 0, dN2dy,
		0, dN2dy, dN2dx,

		dN3dx, 0, dN3dy,
		0, dN3dy, dN3dx,
	};

	for (uint32 i = 0; i < 18; ++i)
	{
		out[i] = B[i];
	}
}

// Polar Decomposition
// "Matrix Animation and Polar Decomposition" by Ken Shoemake, Tom Duff.
static B3_FORCE_INLINE b3Mat22 b3ExtractRotation(const b3Mat22& M)
{
	scalar m11 = M.x.x, m12 = M.y.x;
	scalar m21 = M.x.y, m22 = M.y.y;

	scalar det = m11 * m22 - m12 * m21;

	b3Mat22 A;
	A.x.x = M.y.y;
	A.x.y = -M.y.x;
	A.y.x = -M.x.y;
	A.y.y = M.x.x;

	b3Mat22 Q = M + b3Sign(det) * A;

	Q.x.Normalize();
	Q.y.Normalize();

	return Q;
}

static B3_FORCE_INLINE scalar& b3GetElement(b3Mat22 K[9], uint32 i, uint32 j)
{
	B3_ASSERT(i < 6);
	B3_ASSERT(j < 6);

	uint32 i0 = i / 2;
	uint32 j0 = j / 2;

	b3Mat22& a = K[i0 + 3 * j0];

	uint32 ii = i - 2 * i0;
	uint32 jj = j - 2 * j0;

	return a(ii, jj);
}

// Convert a 6x6 matrix to 2x2 block form.
static B3_FORCE_INLINE void b3SetK(b3Mat22 K[9], scalar Ke[36])
{
	for (uint32 i = 0; i < 6; ++i)
	{
		for (uint32 j = 0; j < 6; ++j)
		{
			scalar k1 = Ke[i + 6 * j];
			scalar& k2 = b3GetElement(K, i, j);

			k2 = k1;
		}
	}
}

b3TriangleElementForce::b3TriangleElementForce(const b3TriangleElementForceDef* def)
{
	m_type = e_triangleElementForce;
	m_userIndex = def->userIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_E_x = def->youngModulusX;
	m_E_y = def->youngModulusY;
	m_E_s = def->shearModulus;
	m_nu_xy = def->poissonRationXY;
	m_nu_yx = def->poissonRationYX;
	m_v1 = def->v1;
	m_v2 = def->v2;
	m_v3 = def->v3;
	m_stiffnessDamping = def->stiffnessDamping;

	ResetElementData();
}

bool b3TriangleElementForce::Contains(const b3Particle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle;
}

void b3TriangleElementForce::ResetElementData()
{
	b3Vec3 p1 = m_v1;
	b3Vec3 p2 = m_v2;
	b3Vec3 p3 = m_v3;

	b3Vec3 n = b3Cross(p2 - p1, p3 - p1);
	n.Normalize();

	// Create a basis B = [px py n] for the plane.  
	// B rotates vectors from plane space to world space.
	b3Vec3 px, py;
	b3ComputeBasis(n, px, py);

	// 2x3	
	scalar P[6] =
	{
		px.x, py.x,
		px.y, py.y,
		px.z, py.z
	};

	b3Vec2 x1, x2, x3;
	b3Mul(&x1.x, P, 2, 3, &p1.x, 3, 1);
	b3Mul(&x2.x, P, 2, 3, &p2.x, 3, 1);
	b3Mul(&x3.x, P, 2, 3, &p3.x, 3, 1);

	// Store initial positions in 2D
	m_x1 = x1;
	m_x2 = x2;
	m_x3 = x3;

	b3Vec2 e1 = x2 - x1;
	b3Vec2 e2 = x3 - x1;

	b3Mat22 S(e1, e2);
	
	scalar det = b3Det(e1, e2);
	B3_ASSERT(det != scalar(0));

	// Area
	scalar A = scalar(0.5) * b3Abs(det);

	// S^-1
	m_invS = det * b3Adjugate(S);

	// 3x3
	b3Mat33 C = b3ComputeC(m_E_x, m_E_y, m_E_s, m_nu_xy, m_nu_yx);

	// 3x6
	scalar B[18];
	b3ComputeB(B, m_invS);

	// 6x3
	scalar BT[18];
	b3Transpose(BT, B, 3, 6);

	// 6x3
	scalar BT_C[18];
	b3Mul(BT_C, BT, 6, 3, &C.x.x, 3, 3);

	// 6x6
	scalar K[36];
	b3Mul(K, BT_C, 6, 3, B, 3, 6);
	for (uint32 i = 0; i < 36; ++i)
	{
		K[i] *= A;
	}

	// Convert to block form
	b3SetK(m_K, K);
}

void b3TriangleElementForce::ClearForces()
{
}

void b3TriangleElementForce::ApplyForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& p = *data->x;
	const b3DenseVec3& v = *data->v;

	b3DenseVec3& f = *data->f;
	
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	uint32 i1 = m_p1->m_solverId;
	uint32 i2 = m_p2->m_solverId;
	uint32 i3 = m_p3->m_solverId;

	b3Vec3 p1 = p[i1];
	b3Vec3 p2 = p[i2];
	b3Vec3 p3 = p[i3];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];

	b3Vec3 n = b3Cross(p2 - p1, p3 - p1);
	n.Normalize();

	b3Vec3 px, py;
	b3ComputeBasis(n, px, py);

	// 2x3
	scalar P[6] =
	{
		px.x, py.x,
		px.y, py.y,
		px.z, py.z
	};

	// 3x2
	scalar PT[6] =
	{
		px.x, px.y, px.z,
		py.x, py.y, py.z
	};

	// Project the positions to 2D
	b3Vec2 x1, x2, x3;
	b3Mul(&x1.x, P, 2, 3, &p1.x, 3, 1);
	b3Mul(&x2.x, P, 2, 3, &p2.x, 3, 1);
	b3Mul(&x3.x, P, 2, 3, &p3.x, 3, 1);

	b3Mat22 T;
	T.x = x2 - x1;
	T.y = x3 - x1;

	// Deformation gradient
	b3Mat22 F = T * m_invS;

	// Extract rotation
	b3Mat22 R = b3ExtractRotation(F);

	// Inverse rotation
	b3Mat22 RT = b3Transpose(R);

	// 2D displacements in unrotated frame
	b3Vec2 us[3];
	us[0] = RT * x1 - m_x1;
	us[1] = RT * x2 - m_x2;
	us[2] = RT * x3 - m_x3;

	// 2D forces in unrotated frame
	b3Vec2 fs[3];

	for (uint32 i = 0; i < 3; ++i)
	{
		fs[i].SetZero();
		
		for (uint32 j = 0; j < 3; ++j)
		{
			b3Mat22 k = m_K[i + 3 * j];

			fs[i] += k * us[j];
		}
	}

	// Rotate the forces to deformed frame
	fs[0] = R * fs[0];
	fs[1] = R * fs[1];
	fs[2] = R * fs[2];

	// Project the forces to 3D
	b3Vec3 f1, f2, f3;
	b3Mul(&f1.x, PT, 3, 2, &fs[0].x, 2, 1);
	b3Mul(&f2.x, PT, 3, 2, &fs[1].x, 2, 1);
	b3Mul(&f3.x, PT, 3, 2, &fs[2].x, 2, 1);

	// Negate f
	f[i1] -= f1;
	f[i2] -= f2;
	f[i3] -= f3;

	// 3D corotated stiffness matrix
	b3Mat33 K[9];

	// K = R * K0 * R^T
	for (uint32 i = 0; i < 3; ++i)
	{
		for (uint32 j = 0; j < 3; ++j)
		{
			// 2D corotated stiffness matrix
			b3Mat22 k = R * m_K[i + 3 * j] * RT;

			// In the paper, Jan uses P^T * R.
			// Here, I use 
			// K = P^T * k * P
			// We can do both ways, but 
			// the latter is more practical than the former.

			// P^T * k = 3x2 * 2x2 = 3x2
			scalar PT_k[6];
			b3Mul(PT_k, PT, 3, 2, &k.x.x, 2, 2);

			// (P^T * k) * P = 3x2 * 2x3 = 3x3
			b3Mat33 Ke;
			b3Mul(&Ke.x.x, PT_k, 3, 2, P, 2, 3);

			K[i + 3 * j] = Ke;
		}
	}

	uint32 is[3] = { i1, i2, i3 };

	for (uint32 i = 0; i < 3; ++i)
	{
		uint32 vi = is[i];

		for (uint32 j = 0; j < 3; ++j)
		{
			uint32 vj = is[j];

			b3Mat33 k = K[i + 3 * j];

			// Negate K 
			dfdx(vi, vj) -= k;
		}
	}

	if (m_stiffnessDamping > scalar(0))
	{
		b3Vec3 vs[3] = { v1, v2, v3 };

		// Lagged Rayleigh damping force
		// fd ~= -k * K(x) * v
		b3Vec3 fds[3];
		for (uint32 i = 0; i < 3; ++i)
		{
			fds[i].SetZero();
			for (uint32 j = 0; j < 3; ++j)
			{
				b3Mat33 k = K[i + 3 * j];

				fds[i] += m_stiffnessDamping * k * vs[j];
			}
		}

		// Negate f
		f[i1] -= fds[0];
		f[i2] -= fds[1];
		f[i3] -= fds[2];

		for (uint32 i = 0; i < 3; ++i)
		{
			uint32 vi = is[i];

			for (uint32 j = 0; j < 3; ++j)
			{
				uint32 vj = is[j];

				b3Mat33 k = K[i + 3 * j];

				// Negate K
				dfdv(vi, vj) -= m_stiffnessDamping * k;
			}
		}
	}
}
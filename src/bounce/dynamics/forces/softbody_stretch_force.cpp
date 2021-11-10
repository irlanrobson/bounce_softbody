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

#include <bounce/dynamics/forces/softbody_stretch_force.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/sparse/sparse_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

// This file contains an implementation for the stretch constraint described 
// in the work of David Baraff and Andrew Witkin: "Large Steps in Cloth Simulation".
// 
// In "A Finite Element Formulation of Baraff-Witkin Cloth" by Theodore Kim, 
// the eigensystem for the stretch energy is revealed and hence the paper describes a way for projecting 
// the negative eigenvalues to positive values. 
// Our projection method is equivalent to Kim's, but a little slower to converge but simpler to implement, as 
// described in the paper "Stable but responsive cloth" of Choi. In practice we just send full force Jacobians 
// to the solver if the eigenvalues are positive.

void b3SoftBodyStretchForceDef::Initialize(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	// (u, v) 1
	u1 = scalar(0);
	v1 = scalar(0);

	// (u, v) 2
	u2 = b3Length(AB);
	v2 = scalar(0);

	// (u, v) 3
	B3_ASSERT(u2 > scalar(0));
	b3Vec3 n_AB = AB / u2;

	// a  = b * h / 2
	// h = (a * 2) / b
	scalar a2 = b3Length(b3Cross(AB, AC));
	B3_ASSERT(a2 > scalar(0));

	u3 = b3Dot(AC, n_AB);
	v3 = a2 / u2;
}

b3SoftBodyStretchForce::b3SoftBodyStretchForce(const b3SoftBodyStretchForceDef* def)
{
	m_type = e_softBodyStretchForce;
	m_meshIndex = def->meshIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_ks_u = def->stretching_stiffness_u;
	m_kd_u = def->damping_stiffness_u;
	m_b_u = def->b_u;
	m_ks_v = def->stretching_stiffness_u;
	m_kd_v = def->damping_stiffness_v;
	m_b_v = def->b_v;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();

	scalar u1 = def->u1, v1 = def->v1;
	scalar u2 = def->u2, v2 = def->v2;
	scalar u3 = def->u3, v3 = def->v3;

	// (u, v) matrix
	scalar du1 = u2 - u1;
	scalar dv1 = v2 - v1;
	scalar du2 = u3 - u1;
	scalar dv2 = v3 - v1;

	m_du1 = du1;
	m_dv1 = dv1;
	m_du2 = du2;
	m_dv2 = dv2;

	scalar det = du1 * dv2 - du2 * dv1;
	B3_ASSERT(det != scalar(0));
	m_inv_det = scalar(1) / det;

	m_dwudx.x = m_inv_det * (dv1 - dv2);
	m_dwudx.y = m_inv_det * dv2;
	m_dwudx.z = -m_inv_det * dv1;

	m_dwvdx.x = m_inv_det * (du2 - du1);
	m_dwvdx.y = -m_inv_det * du2;
	m_dwvdx.z = m_inv_det * du1;

	m_alpha = scalar(0.5) * b3Abs(det);
}

b3SoftBodyStretchForce::~b3SoftBodyStretchForce()
{

}

bool b3SoftBodyStretchForce::HasParticle(const b3SoftBodyParticle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle;
}

void b3SoftBodyStretchForce::ComputeForces(const b3SparseForceSolverData* data)
{
	scalar alpha = m_alpha;
	scalar du1 = m_du1;
	scalar dv1 = m_dv1;
	scalar du2 = m_du2;
	scalar dv2 = m_dv2;
	scalar inv_det = m_inv_det;
	b3Vec3 dwudx = m_dwudx;
	b3Vec3 dwvdx = m_dwvdx;

	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;
	u32 i3 = m_p3->m_solverId;

	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3Vec3 x1 = x[i1];
	b3Vec3 x2 = x[i2];
	b3Vec3 x3 = x[i3];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];

	b3Mat33 I; I.SetIdentity();

	b3Vec3 dx1 = x2 - x1;
	b3Vec3 dx2 = x3 - x1;

	b3Vec3 wu = inv_det * (dv2 * dx1 - dv1 * dx2);
	scalar len_wu = b3Length(wu);

	b3Vec3 wv = inv_det * (-du2 * dx1 + du1 * dx2);
	scalar len_wv = b3Length(wv);

	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();

	if (len_wu > scalar(0))
	{
		scalar inv_len_wu = scalar(1) / len_wu;
		b3Vec3 n_wu = inv_len_wu * wu;

		// Jacobian
		b3Vec3 dCudx[3];
		for (u32 i = 0; i < 3; ++i)
		{
			dCudx[i] = alpha * dwudx[i] * n_wu;
		}

		if (m_ks_u > scalar(0))
		{
			scalar Cu = alpha * (len_wu - m_b_u);

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_ks_u * Cu * dCudx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Force derivative
			b3Mat33 K[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Kij = b3Outer(dCudx[i], dCudx[j]);

					// Are the eigenvalues positive?
					if (len_wu > m_b_u)
					{
						b3Mat33 d2Cuxij = (alpha * inv_len_wu * dwudx[i] * dwudx[j]) * (I - b3Outer(n_wu, n_wu));

						Kij += Cu * d2Cuxij;
					}

					K[i][j] = -m_ks_u * Kij;
				}
			}

			dfdx(i1, i1) += K[0][0];
			dfdx(i1, i2) += K[0][1];
			dfdx(i1, i3) += K[0][2];

			dfdx(i2, i1) += K[1][0];
			dfdx(i2, i2) += K[1][1];
			dfdx(i2, i3) += K[1][2];

			dfdx(i3, i1) += K[2][0];
			dfdx(i3, i2) += K[2][1];
			dfdx(i3, i3) += K[2][2];
		}

		if (m_kd_u > scalar(0))
		{
			b3Vec3 vs[3] = { v1, v2, v3 };
			scalar dCudt = scalar(0);
			for (u32 i = 0; i < 3; ++i)
			{
				dCudt += b3Dot(dCudx[i], vs[i]);
			}

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_kd_u * dCudt * dCudx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Force derivative
			b3Mat33 K[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Kij = -m_kd_u * b3Outer(dCudx[i], dCudx[j]);

					K[i][j] = Kij;
				}
			}

			dfdv(i1, i1) += K[0][0];
			dfdv(i1, i2) += K[0][1];
			dfdv(i1, i3) += K[0][2];

			dfdv(i2, i1) += K[1][0];
			dfdv(i2, i2) += K[1][1];
			dfdv(i2, i3) += K[1][2];

			dfdv(i3, i1) += K[2][0];
			dfdv(i3, i2) += K[2][1];
			dfdv(i3, i3) += K[2][2];
		}
	}

	if (len_wv > scalar(0))
	{
		scalar inv_len_wv = scalar(1) / len_wv;
		b3Vec3 n_wv = inv_len_wv * wv;

		// Jacobian
		b3Vec3 dCvdx[3];
		for (u32 i = 0; i < 3; ++i)
		{
			dCvdx[i] = alpha * dwvdx[i] * n_wv;
		}

		if (m_ks_v > scalar(0))
		{
			scalar Cv = alpha * (len_wv - m_b_v);

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_ks_v * Cv * dCvdx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Force derivative
			b3Mat33 K[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Kij = b3Outer(dCvdx[i], dCvdx[j]);

					// Are the eigenvalues positive?
					if (len_wv > m_b_v)
					{
						b3Mat33 d2Cvxij = (alpha * inv_len_wv * dwvdx[i] * dwvdx[j]) * (I - b3Outer(n_wv, n_wv));

						Kij += Cv * d2Cvxij;
					}

					K[i][j] = -m_ks_v * Kij;
				}
			}

			dfdx(i1, i1) += K[0][0];
			dfdx(i1, i2) += K[0][1];
			dfdx(i1, i3) += K[0][2];

			dfdx(i2, i1) += K[1][0];
			dfdx(i2, i2) += K[1][1];
			dfdx(i2, i3) += K[1][2];

			dfdx(i3, i1) += K[2][0];
			dfdx(i3, i2) += K[2][1];
			dfdx(i3, i3) += K[2][2];
		}

		if (m_kd_v > scalar(0))
		{
			b3Vec3 vs[3] = { v1, v2, v3 };

			scalar dCvdt = scalar(0);
			for (u32 i = 0; i < 3; ++i)
			{
				dCvdt += b3Dot(dCvdx[i], vs[i]);
			}

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_kd_v * dCvdt * dCvdx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Force derivative
			b3Mat33 K[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Kij = -m_kd_v * b3Outer(dCvdx[i], dCvdx[j]);

					K[i][j] = Kij;
				}
			}

			dfdv(i1, i1) += K[0][0];
			dfdv(i1, i2) += K[0][1];
			dfdv(i1, i3) += K[0][2];

			dfdv(i2, i1) += K[1][0];
			dfdv(i2, i2) += K[1][1];
			dfdv(i2, i3) += K[1][2];

			dfdv(i3, i1) += K[2][0];
			dfdv(i3, i2) += K[2][1];
			dfdv(i3, i3) += K[2][2];
		}
	}

	f[i1] += m_f1;
	f[i2] += m_f2;
	f[i3] += m_f3;
}
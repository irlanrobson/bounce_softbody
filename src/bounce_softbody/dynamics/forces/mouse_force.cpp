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

#include <bounce_softbody/dynamics/forces/mouse_force.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/sparse_mat33.h>

b3MouseForce::b3MouseForce(const b3MouseForceDef* def)
{
	m_type = e_mouseForce;
	m_userIndex = def->userIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_p4 = def->p4;
	m_w2 = def->w2;
	m_w3 = def->w3;
	m_w4 = def->w4;
	m_ks = def->stiffness;
	m_kd = def->dampingStiffness;
	m_L0 = def->length;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
	m_f4.SetZero();
}

bool b3MouseForce::HasParticle(const b3Particle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle || m_p4 == particle;
}

void b3MouseForce::ClearForces()
{
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
	m_f4.SetZero();
}

void b3MouseForce::ComputeForces(const b3SparseForceSolverData* data)
{
	uint32 i1 = m_p1->m_solverId;
	uint32 i2 = m_p2->m_solverId;
	uint32 i3 = m_p3->m_solverId;
	uint32 i4 = m_p4->m_solverId;

	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3Vec3 x1 = x[i1];
	b3Vec3 x2 = x[i2];
	b3Vec3 x3 = x[i3];
	b3Vec3 x4 = x[i4];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];
	b3Vec3 v4 = v[i4];

	b3Mat33 I; I.SetIdentity();

	scalar w2 = m_w2;
	scalar w3 = m_w3;
	scalar w4 = m_w4;

	b3Vec3 c2 = w2 * x2 + w3 * x3 + w4 * x4;

	b3Vec3 d = x1 - c2;
	scalar L = b3Length(d);

	if (L > scalar(0))
	{
		scalar inv_L = scalar(1) / L;

		b3Vec3 n = inv_L * d;

		// Jacobian
		b3Vec3 dCdx[4];
		dCdx[0] = n;
		dCdx[1] = -w2 * n;
		dCdx[2] = -w3 * n;
		dCdx[3] = -w4 * n;

		if (m_ks > scalar(0))
		{
			if (L > m_L0)
			{
				scalar C = L - m_L0;

				// Force
				b3Vec3 fs[4];
				for (uint32 i = 0; i < 4; ++i)
				{
					fs[i] = -m_ks * C * dCdx[i];
				}

				f[i1] += fs[0];
				f[i2] += fs[1];
				f[i3] += fs[2];
				f[i4] += fs[3];

				m_f1 += fs[0];
				m_f2 += fs[1];
				m_f3 += fs[2];
				m_f4 += fs[3];

				// Force derivative
				scalar L3 = L * L * L;

				scalar inv_L3 = L3 > scalar(0) ? scalar(1) / L3 : scalar(0);

				b3Mat33 d2Cdx[4][4];
				
				b3Mat33 A = inv_L * I - inv_L3 * b3Outer(d, d);

				d2Cdx[0][0] = A;
				d2Cdx[0][1] = -w2 * A;
				d2Cdx[0][2] = -w3 * A;
				d2Cdx[0][3] = -w4 * A;

				b3Mat33 B = inv_L3 * b3Outer(d, d) - inv_L * I;

				d2Cdx[1][0] = w2 * B;
				d2Cdx[1][1] = -w2 * w2 * B;
				d2Cdx[1][2] = -w2 * w3 * B;
				d2Cdx[1][3] = -w2 * w4 * B;

				d2Cdx[2][0] = w3 * B;
				d2Cdx[2][1] = -w2 * w3 * B;
				d2Cdx[2][2] = -w3 * w3 * B;
				d2Cdx[2][3] = -w3 * w4 * B;

				d2Cdx[3][0] = w4 * B;
				d2Cdx[3][1] = -w2 * w4 * B;
				d2Cdx[3][2] = -w3 * w4 * B;
				d2Cdx[3][3] = -w4 * w4 * B;

				b3Mat33 K[4][4];
				for (uint32 i = 0; i < 4; ++i)
				{
					for (uint32 j = 0; j < 4; ++j)
					{
						K[i][j] = -m_ks * (b3Outer(dCdx[i], dCdx[j]) + C * d2Cdx[i][j]);
					}
				}

				dfdx(i1, i1) += K[0][0];
				dfdx(i1, i2) += K[0][1];
				dfdx(i1, i3) += K[0][2];
				dfdx(i1, i4) += K[0][3];

				dfdx(i2, i1) += K[1][0];
				dfdx(i2, i2) += K[1][1];
				dfdx(i2, i3) += K[1][2];
				dfdx(i2, i4) += K[1][3];

				dfdx(i3, i1) += K[2][0];
				dfdx(i3, i2) += K[2][1];
				dfdx(i3, i3) += K[2][2];
				dfdx(i3, i4) += K[2][3];

				dfdx(i4, i1) += K[3][0];
				dfdx(i4, i2) += K[3][1];
				dfdx(i4, i3) += K[3][2];
				dfdx(i4, i4) += K[3][3];
			}
		}

		if (m_kd > scalar(0))
		{
			b3Vec3 vc2 = m_w2 * v2 + m_w3 * v3 + m_w4 * v4;

			scalar dCdt = b3Dot(v1 - vc2, n);
			
			// Force
			b3Vec3 fs[4];
			for (uint32 i = 0; i < 4; ++i)
			{
				fs[i] = -m_kd * dCdt * dCdx[i];
			}

			f[i1] += fs[0];
			f[i2] += fs[1];
			f[i3] += fs[2];
			f[i4] += fs[3];

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];
			m_f4 += fs[3];

			// Force derivative
			b3Mat33 K[4][4];
			for (uint32 i = 0; i < 4; ++i)
			{
				for (uint32 j = 0; j < 4; ++j)
				{
					b3Mat33 Kij = -m_kd * b3Outer(dCdx[i], dCdx[j]);

					K[i][j] = Kij;
				}
			}

			dfdv(i1, i1) += K[0][0];
			dfdv(i1, i2) += K[0][1];
			dfdv(i1, i3) += K[0][2];
			dfdv(i1, i4) += K[0][3];

			dfdv(i2, i1) += K[1][0];
			dfdv(i2, i2) += K[1][1];
			dfdv(i2, i3) += K[1][2];
			dfdv(i2, i4) += K[1][3];

			dfdv(i3, i1) += K[2][0];
			dfdv(i3, i2) += K[2][1];
			dfdv(i3, i3) += K[2][2];
			dfdv(i3, i4) += K[2][3];

			dfdv(i4, i1) += K[3][0];
			dfdv(i4, i2) += K[3][1];
			dfdv(i4, i3) += K[3][2];
			dfdv(i4, i4) += K[3][3];
		}
	}
}
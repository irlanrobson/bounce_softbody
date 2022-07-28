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

#include <bounce_softbody/dynamics/forces/spring_force.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/sparse_mat33.h>

void b3SpringForceDef::Initialize(b3Particle* particle1, b3Particle* particle2, scalar ks, scalar kd)
{
	type = e_springForce;
	p1 = particle1;
	p2 = particle2;
	b3Vec3 x1 = p1->GetPosition();
	b3Vec3 x2 = p2->GetPosition();
	length = b3Distance(x1, x2);
	stiffness = ks;
	dampingStiffness = kd;
}

b3SpringForce::b3SpringForce(const b3SpringForceDef* def)
{
	m_type = e_springForce;
	m_userIndex = def->userIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_L0 = def->length;
	m_ks = def->stiffness;
	m_kd = def->dampingStiffness;
	m_f1.SetZero();
	m_f2.SetZero();
}

bool b3SpringForce::Contains(const b3Particle* particle) const
{
	return m_p1 == particle || m_p2 == particle;
}

void b3SpringForce::ClearForces()
{
	m_f1.SetZero();
	m_f2.SetZero();
}

void b3SpringForce::ApplyForces(const b3SparseForceSolverData* data)
{
	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	uint32 i1 = m_p1->m_solverId;
	uint32 i2 = m_p2->m_solverId;

	b3Vec3 x1 = x[i1];
	b3Vec3 v1 = v[i1];

	b3Vec3 x2 = x[i2];
	b3Vec3 v2 = v[i2];

	b3Mat33 I; I.SetIdentity();

	b3Vec3 dx = x1 - x2;

	scalar L = b3Length(dx);

	if (L > scalar(0))
	{
		b3Vec3 n = dx / L;

		if (m_ks > scalar(0))
		{
			if (L > m_L0)
			{
				scalar C = L - m_L0;

				// Force
				b3Vec3 f1 = -m_ks * C * n;
				b3Vec3 f2 = -f1;

				f[i1] += f1;
				f[i2] += f2;

				m_f1 += f1;
				m_f2 += f2;

				// Force derivative
				b3Mat33 K11 = -m_ks * (b3Outer(dx, dx) + (scalar(1) - m_L0 / L) * (I - b3Outer(dx, dx)));
				b3Mat33 K12 = -K11;
				b3Mat33 K21 = K12;
				b3Mat33 K22 = K11;

				dfdx(i1, i1) += K11;
				dfdx(i1, i2) += K12;
				dfdx(i2, i1) += K21;
				dfdx(i2, i2) += K22;
			}
		}

		if (m_kd > scalar(0))
		{
			scalar dCdt = b3Dot(n, v1 - v2);

			// Force
			b3Vec3 f1 = -m_kd * dCdt * n;
			b3Vec3 f2 = -f1;

			f[i1] += f1;
			f[i2] += f2;

			m_f1 += f1;
			m_f2 += f2;

			// Force derivative
			b3Mat33 K11 = -m_kd * b3Outer(n, n);
			b3Mat33 K12 = -K11;
			b3Mat33 K21 = K12;
			b3Mat33 K22 = K11;

			dfdv(i1, i1) += K11;
			dfdv(i1, i2) += K12;
			dfdv(i2, i1) += K21;
			dfdv(i2, i2) += K22;
		}
	}
}
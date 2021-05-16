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

#include <bounce/dynamics/forces/softbody_spring_force.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/sparse/sparse_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

void b3SoftBodySpringForceDef::Initialize(b3SoftBodyParticle* particle1, b3SoftBodyParticle* particle2, scalar structuralStiffness, scalar structuralDampingStiffness)
{
	type = e_softBodySpringForce;
	p1 = particle1;
	p2 = particle2;
	b3Vec3 x1 = p1->GetPosition();
	b3Vec3 x2 = p2->GetPosition();
	restLength = b3Distance(x1, x2);
	stiffness = structuralStiffness;
	dampingStiffness = structuralDampingStiffness;
}

b3SoftBodySpringForce::b3SoftBodySpringForce(const b3SoftBodySpringForceDef* def)
{
	m_type = e_softBodySpringForce;
	m_meshIndex = def->meshIndex;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_L0 = def->restLength;
	m_ks = def->stiffness;
	m_kd = def->dampingStiffness;
	m_f1.SetZero();
	m_f2.SetZero();
}

b3SoftBodySpringForce::~b3SoftBodySpringForce()
{

}

bool b3SoftBodySpringForce::HasParticle(const b3SoftBodyParticle* particle) const
{
	return m_p1 == particle || m_p2 == particle;
}

void b3SoftBodySpringForce::ComputeForces(const b3SparseForceSolverData* data)
{
	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;

	b3Vec3 x1 = x[i1];
	b3Vec3 v1 = v[i1];

	b3Vec3 x2 = x[i2];
	b3Vec3 v2 = v[i2];

	b3Mat33 I; I.SetIdentity();

	m_f1.SetZero();
	m_f2.SetZero();

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

				m_f1 += -m_ks * C * n;
				m_f2 -= -m_ks * C * n;

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
			m_f1 += -m_kd * dCdt * n;
			m_f2 -= -m_kd * dCdt * n;

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

	f[i1] += m_f1;
	f[i2] += m_f2;
}
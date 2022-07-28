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

#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/dynamics/contacts/sphere_shape_contact.h>
#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/triangle_fixture.h>
#include <bounce_softbody/dynamics/fixtures/tetrahedron_fixture.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/sparse_mat33.h>

b3Particle::b3Particle(const b3ParticleDef& def, b3Body* body)
{
	m_body = body;
	m_type = def.type;
	m_position = def.position;
	m_velocity = def.velocity;
	m_force.SetZero();
	m_translation.SetZero();
	m_damping = def.damping;

	if (m_type == e_dynamicParticle)
	{
		m_mass = scalar(1);
		m_invMass = scalar(1);
	}
	else
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}

	m_userIndex = def.userIndex;
	m_userData = def.userData;
	
	m_prev = nullptr;
	m_next = nullptr;
}

void b3Particle::SetType(b3ParticleType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;

	if (m_type == e_staticParticle || m_type == e_kinematicParticle)
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}
	else
	{
		m_body->ResetMass();
	}

	m_force.SetZero();
	m_translation.SetZero();

	if (type == e_staticParticle)
	{
		m_velocity.SetZero();
		SynchronizeFixtures();
	}

	DestroyContacts();
}

void b3Particle::DestroyFixtures()
{
	{
		// Destroy spheres
		b3SphereFixture* s = m_body->m_sphereList;
		while (s)
		{
			b3SphereFixture* s0 = s;
			s = s->m_next;

			if (s0->m_p == this)
			{
				m_body->DestroySphere(s0);
			}
		}
	}

	{
		// Destroy triangles
		b3TriangleFixture* t = m_body->m_triangleList;
		while (t)
		{
			b3TriangleFixture* t0 = t;
			t = t->m_next;

			if (t0->m_p1 == this || t0->m_p2 == this || t0->m_p3 == this)
			{
				m_body->DestroyTriangle(t0);
			}
		}
	}

	{
		// Destroy tetrahedrons
		b3TetrahedronFixture* t = m_body->m_tetrahedronList;
		while (t)
		{
			b3TetrahedronFixture* t0 = t;
			t = t->m_next;

			if (t0->m_p1 == this || t0->m_p2 == this || t0->m_p3 == this || t0->m_p4 == this)
			{
				m_body->DestroyTetrahedron(t0);
			}
		}
	}
}

void b3Particle::DestroyForces()
{
	b3Force* f = m_body->m_forceList;
	while (f)
	{
		b3Force* f0 = f;
		f = f->m_next;

		if (f0->Contains(this))
		{
			m_body->DestroyForce(f0);
		}
	}
}

void b3Particle::DestroyContacts()
{
	// Destroy shape contacts
	b3SphereAndShapeContact* c = m_body->m_contactManager.m_contactList;
	while (c)
	{
		if (c->m_fixture1->m_p == this)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			m_body->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}

void b3Particle::SynchronizeFixtures()
{
	// Synchronize triangles
	for (b3TriangleFixture* t = m_body->m_triangleList; t; t = t->m_next)
	{
		if (t->m_p1 == this || t->m_p2 == this || t->m_p3 == this)
		{
			t->Synchronize(b3Vec3_zero);
		}
	}
}

void b3Particle::ApplyForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdv = *data->dfdv;

	uint32 i = m_solverId;

	if (m_damping > scalar(0))
	{
		b3Vec3 fd = -m_damping * m_mass * v[i];

		// Mass damping force
		f[i] += fd;

		// Jacobian
		dfdv(i, i) += b3Mat33Diagonal(-m_damping * m_mass);
	}
}
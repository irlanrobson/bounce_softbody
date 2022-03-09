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

#include <bounce_softbody/dynamics/friction_solver.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/contacts/sphere_shape_contact.h>
#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/common/math/vec2.h>
#include <bounce_softbody/common/memory/stack_allocator.h>

b3FrictionSolver::b3FrictionSolver(const b3FrictionSolverDef& def)
{
	m_step = def.step;
	m_shapeContactCount = def.shapeContactCount;
	m_shapeContacts = def.shapeContacts;
}

void b3FrictionSolver::Solve()
{
	for (uint32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SphereAndShapeContact* c = m_shapeContacts[i];
		if (c->m_active == false)
		{
			continue;
		}

		b3SphereFixture* f1 = c->m_f1;
		b3Particle* p1 = f1->m_p;
		b3WorldFixture* f2 = c->m_f2;

		b3Vec3 v1 = p1->m_velocity;
		scalar m1 = p1->m_invMass;
		
		b3Vec3 tangent1 = c->m_tangent1;
		b3Vec3 tangent2 = c->m_tangent2;
		scalar normalForce = c->m_normalForce;
		
		scalar friction = b3MixFriction(f1->m_friction, f2->m_friction);

		// Compute effective mass.
		scalar tangentMass = m1 > scalar(0) ? scalar(1) / m1 : scalar(0);

		b3Vec2 Cdot;
		Cdot.x = b3Dot(v1, tangent1);
		Cdot.y = b3Dot(v1, tangent2);

		b3Vec2 impulse = tangentMass * -Cdot;
		scalar normalImpulse = m_step.dt * normalForce;

		scalar maxImpulse = friction * normalImpulse;
		if (b3Dot(impulse, impulse) > maxImpulse * maxImpulse)
		{
			impulse.Normalize();
			impulse *= maxImpulse;
		}

		b3Vec3 P1 = impulse.x * tangent1;
		b3Vec3 P2 = impulse.y * tangent2;
		b3Vec3 P = P1 + P2;

		v1 += m1 * P;

		p1->m_velocity = v1;
	}
}
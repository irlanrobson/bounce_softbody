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

#include <bounce_softbody/dynamics/contacts/sphere_shape_contact.h>
#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/time_step.h>
#include <bounce_softbody/collision/geometry/sphere.h>
#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/sparse_mat33.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/common/math/vec2.h>
#include <bounce_softbody/common/memory/block_allocator.h>

b3SphereAndShapeContact* b3SphereAndShapeContact::Create(b3SphereFixture* fixture1, b3WorldFixture* fixture2, b3BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b3SphereAndShapeContact));
	return new(mem) b3SphereAndShapeContact(fixture1, fixture2);
}

void b3SphereAndShapeContact::Destroy(b3SphereAndShapeContact* contact, b3BlockAllocator* allocator)
{
	contact->~b3SphereAndShapeContact();
	allocator->Free(contact, sizeof(b3SphereAndShapeContact));
}

b3SphereAndShapeContact::b3SphereAndShapeContact(b3SphereFixture* fixture1, b3WorldFixture* fixture2)
{
	m_prev = nullptr;
	m_next = nullptr;
	m_fixture1 = fixture1;
	m_fixture2 = fixture2;
	m_normalForce = scalar(0);
	m_applyFriction = false;
}

void b3SphereAndShapeContact::ApplyForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& x = *data->x;
	const b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3Particle* p1 = m_fixture1->m_p;

	uint32 i1 = p1->m_solverId;

	b3Vec3 x1 = x[i1];
	b3Vec3 v1 = v[i1];

	scalar r1 = m_fixture1->m_radius;
	scalar r2 = m_fixture2->m_shape->m_radius;

	b3Sphere sphere1;
	sphere1.vertex = x1;
	sphere1.radius = r1;

	// Evaluate the contact manifold.
	b3SphereManifold manifold2;
	if (m_fixture2->Collide(&manifold2, sphere1) == false)
	{
		return;
	}

	// The friction solver uses initial tangents.
	if (m_applyFriction == false)
	{
		m_manifold = manifold2;
		m_applyFriction = true;
	}

	b3Vec3 x2 = manifold2.point;
	b3Vec3 n2 = manifold2.normal;

	// Force computation requires normal direction from fixture 1 to fixture 2.
	b3Vec3 n1 = -n2;

	// Theodore Kim and David Eberle:
	// "Dynamic Deformables: Implementation and Production Practicalities", page 143.

	// Apply normal force.
	if (B3_CONTACT_STIFFNESS > scalar(0))
	{
		// Closest points on the surface of the fixtures.
		b3Vec3 c1 = x1 + r1 * n1;
		b3Vec3 c2 = x2 + r2 * n2;

		// There is no spring rest length.
		// Therefore, there is no compression force.
		scalar C = b3Length(c2 - c1);

		// Clamp correction to prevent large forces.
		C = b3Min(B3_BAUMGARTE * C, B3_MAX_CONTACT_LINEAR_CORRECTION);

		// Spring force
		b3Vec3 f1 = -B3_CONTACT_STIFFNESS * C * n1;

		b3Mat33 I = b3Mat33_identity;

		// Jacobian
		b3Mat33 K11 = -B3_CONTACT_STIFFNESS * (b3Outer(n1, n1) + C * (I - b3Outer(n1, n1)));

		// Apply 
		f[i1] += f1;
		dfdx(i1, i1) += K11;

		// Accumulate normal force magnitude for friction.
		m_normalForce += b3Length(f1);
	}

	// Apply damping force.
	if (B3_CONTACT_DAMPING_STIFFNESS > scalar(0))
	{
		scalar dCdt = b3Dot(v1, n1);

		// Damping force
		b3Vec3 f1 = -B3_CONTACT_DAMPING_STIFFNESS * dCdt * n1;

		// Jacobian
		b3Mat33 K11 = -B3_CONTACT_DAMPING_STIFFNESS * b3Outer(n1, n1);

		// Apply force and Jacobian
		f[i1] += f1;
		dfdv(i1, i1) += K11;
	}
}

void b3SphereAndShapeContact::ApplyFriction(const b3TimeStep& step, const b3Vec3& gravity)
{
	if (m_applyFriction == false)
	{
		return;
	}

	b3Vec3 normal = m_manifold.normal;
	scalar normalForce = m_normalForce;

	b3Vec3 tangent1 = b3Perp(normal);
	b3Vec3 tangent2 = b3Cross(tangent1, normal);

	b3Particle* p1 = m_fixture1->m_p;
	
	b3Vec3 v1 = p1->m_velocity;
	scalar im1 = p1->m_invMass;

	scalar friction = b3MixFriction(m_fixture1->m_friction, m_fixture2->m_friction);

	// Compute effective mass.
	scalar tangentMass = im1 > scalar(0) ? scalar(1) / im1 : scalar(0);

	b3Vec2 Cdot;
	Cdot.x = b3Dot(v1, tangent1);
	Cdot.y = b3Dot(v1, tangent2);

	b3Vec2 impulse = tangentMass * -Cdot;
	scalar normalImpulse = step.dt * normalForce;

	scalar maxImpulse = friction * normalImpulse;
	if (b3Dot(impulse, impulse) > maxImpulse * maxImpulse)
	{
		impulse.Normalize();
		impulse *= maxImpulse;
	}

	b3Vec3 P1 = impulse.x * tangent1;
	b3Vec3 P2 = impulse.y * tangent2;
	b3Vec3 P = P1 + P2;

	v1 += im1 * P;

	p1->m_velocity = v1;
}

void b3SphereAndShapeContact::Update()
{
	m_normalForce = scalar(0);
	m_applyFriction = false;
}
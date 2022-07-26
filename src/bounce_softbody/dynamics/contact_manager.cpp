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

#include <bounce_softbody/dynamics/contact_manager.h>
#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/dynamics/contacts/sphere_shape_contact.h>
#include <bounce_softbody/common/memory/block_allocator.h>

b3ContactManager::b3ContactManager()
{
	m_contactList = nullptr;
	m_contactCount = 0;
}

void b3ContactManager::AddPair(b3SphereFixture* fixture1, b3WorldFixture* fixture2)
{
	// Check if there is a contact between the two entities.
	for (b3SphereAndShapeContact* c = m_contactList; c; c = c->m_next)
	{
		if (c->m_fixture1 == fixture1 && c->m_fixture2 == fixture2)
		{
			// A contact already exists.
			return;
		}
	}

	// Should the entities collide with each other?
	if (fixture1->m_p->m_type != e_dynamicParticle)
	{
		return;
	}

	// Call the factory.
	b3SphereAndShapeContact* c = b3SphereAndShapeContact::Create(fixture1, fixture2, m_allocator);

	// Push the contact to the contact list.
	c->m_prev = nullptr;
	c->m_next = m_contactList;
	if (m_contactList)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;
	++m_contactCount;
}

void b3ContactManager::FindNewContacts()
{
	// Run a simple broadphase loop.
	for (b3SphereFixture* f1 = m_body->m_sphereList; f1; f1 = f1->m_next)
	{
		b3AABB aabb1 = f1->ComputeAABB();

		for (b3WorldFixture* f2 = m_body->m_fixtureList; f2; f2 = f2->m_next)
		{
			b3AABB aabb2 = f2->ComputeAABB();

			if (b3TestOverlap(aabb1, aabb2))
			{
				AddPair(f1, f2);
			}
		}
	}
}

void b3ContactManager::Destroy(b3SphereAndShapeContact* c)
{
	// Remove from the body.
	if (c->m_prev)
	{
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next)
	{
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList)
	{
		m_contactList = c->m_next;
	}

	--m_contactCount;

	// Call the factory.
	b3SphereAndShapeContact::Destroy(c, m_allocator);
}

void b3ContactManager::UpdateContacts()
{
	// Update the state of sphere and shape contacts.
	b3SphereAndShapeContact* c = m_contactList;
	while (c)
	{
		b3SphereFixture* f1 = c->m_fixture1;
		b3Particle* p1 = f1->m_p;

		b3WorldFixture* f2 = c->m_fixture2;

		// Cease the contact if entities must not collide with each other.
		if (p1->m_type != e_dynamicParticle)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		b3AABB aabb1 = f1->ComputeAABB();
		b3AABB aabb2 = f2->ComputeAABB();

		// Destroy the contact if AABBs are not overlapping.
		bool overlap = b3TestOverlap(aabb1, aabb2);
		if (overlap == false)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update();

		c = c->m_next;
	}
}
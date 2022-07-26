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

#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/dynamics/contacts/sphere_shape_contact.h>

b3SphereFixture::b3SphereFixture(const b3SphereFixtureDef& def, b3Body* body) : b3Fixture(def, body)
{
	m_type = e_sphereFixture;
	m_p = def.p;
	m_prev = nullptr;
	m_next = nullptr;
}

b3AABB b3SphereFixture::ComputeAABB() const
{
	return b3AABB(m_p->m_position, m_radius);
}

void b3SphereFixture::DestroyContacts()
{
	b3SphereAndShapeContact* c = m_body->m_contactManager.m_contactList;
	while (c)
	{
		b3SphereAndShapeContact* c0 = c;
		c = c->m_next;

		if (c0->m_fixture1 == this)
		{
			m_body->m_contactManager.Destroy(c0);
			continue;
		}
	}
}
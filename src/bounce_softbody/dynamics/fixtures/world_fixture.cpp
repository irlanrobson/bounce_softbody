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

#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/collision/shapes/sphere_shape.h>
#include <bounce_softbody/collision/shapes/capsule_shape.h>
#include <bounce_softbody/collision/shapes/triangle_shape.h>
#include <bounce_softbody/collision/shapes/box_shape.h>
#include <bounce_softbody/collision/shapes/mesh_shape.h>
#include <bounce_softbody/collision/shapes/sdf_shape.h>
#include <bounce_softbody/common/memory/block_allocator.h>

b3WorldFixture::b3WorldFixture()
{
	m_shape = nullptr;
	m_body = nullptr;
	m_prev = nullptr;
	m_next = nullptr;
	m_friction = scalar(0);
}

void b3WorldFixture::Create(b3BlockAllocator* allocator, b3Body* body, const b3WorldFixtureDef& def)
{
	m_body = body;
	m_friction = def.friction;
	m_shape = def.shape->Clone(allocator);
}

void b3WorldFixture::Destroy(b3BlockAllocator* allocator)
{
	switch (m_shape->m_type)
	{
	case b3Shape::e_sphere:
	{
		b3SphereShape* s = (b3SphereShape*)m_shape;
		s->~b3SphereShape();
		allocator->Free(s, sizeof(b3SphereShape));
		break;
	}
	case b3Shape::e_capsule:
	{
		b3CapsuleShape* s = (b3CapsuleShape*)m_shape;
		s->~b3CapsuleShape();
		allocator->Free(s, sizeof(b3CapsuleShape));
		break;
	}
	case b3Shape::e_triangle:
	{
		b3TriangleShape* s = (b3TriangleShape*)m_shape;
		s->~b3TriangleShape();
		allocator->Free(s, sizeof(b3TriangleShape));
		break;
	}
	case b3Shape::e_box:
	{
		b3BoxShape* s = (b3BoxShape*)m_shape;
		s->~b3BoxShape();
		allocator->Free(s, sizeof(b3BoxShape));
		break;
	}
	case b3Shape::e_mesh:
	{
		b3MeshShape* s = (b3MeshShape*)m_shape;
		s->~b3MeshShape();
		allocator->Free(s, sizeof(b3MeshShape));
		break;
	}
	case b3Shape::e_sdf:
	{
		b3SDFShape* s = (b3SDFShape*)m_shape;
		s->~b3SDFShape();
		allocator->Free(s, sizeof(b3SDFShape));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}

	m_shape = nullptr;
}

void b3WorldFixture::DestroyContacts()
{
	b3SphereAndShapeContact* c = m_body->m_contactManager.m_shapeContactList;
	while (c)
	{
		b3SphereAndShapeContact* c0 = c;
		c = c->m_next;

		if (c0->m_f2 == this)
		{
			m_body->m_contactManager.Destroy(c0);
		}
	}
}
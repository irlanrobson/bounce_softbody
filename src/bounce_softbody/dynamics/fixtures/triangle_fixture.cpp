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

#include <bounce_softbody/dynamics/fixtures/triangle_fixture.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/collision/shapes/triangle_shape.h>

b3TriangleFixture::b3TriangleFixture(const b3TriangleFixtureDef& def, b3Body* body) : b3Fixture(def, body)
{
	m_type = e_triangleFixture;
	
	m_p1 = def.p1;
	m_p2 = def.p2;
	m_p3 = def.p3;
	
	b3Vec3 A = def.v1;
	b3Vec3 B = def.v2;
	b3Vec3 C = def.v3;
	b3Vec3 N = b3Cross(B - A, C - A);
	
	m_area = scalar(0.5) * b3Length(N);

	m_prev = nullptr;
	m_next = nullptr;
}

b3AABB b3TriangleFixture::ComputeAABB() const
{
	b3AABB aabb;
	aabb.lowerBound = b3Min(m_p1->m_position, b3Min(m_p2->m_position, m_p3->m_position));
	aabb.upperBound = b3Max(m_p1->m_position, b3Max(m_p2->m_position, m_p3->m_position));
	aabb.Extend(m_radius);
	return aabb;
}

void b3TriangleFixture::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_body->m_tree.MoveProxy(m_proxyId, aabb, displacement);
}

bool b3TriangleFixture::RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
{
	b3TriangleShape triangle;
	triangle.m_radius = m_radius;
	triangle.m_vertex1 = m_p1->m_position;
	triangle.m_vertex2 = m_p2->m_position;
	triangle.m_vertex3 = m_p3->m_position;
	return triangle.RayCast(output, input);
}
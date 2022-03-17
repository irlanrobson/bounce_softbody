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

#include <bounce_softbody/collision/shapes/capsule_shape.h>
#include <bounce_softbody/collision/geometry/sphere.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

b3CapsuleShape::b3CapsuleShape()
{
	m_type = e_capsule;
	m_center1.Set(0, 1, 0);
	m_center2.Set(0, -1, 0);
	m_radius = scalar(0);
}

b3Shape* b3CapsuleShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3CapsuleShape));
	b3CapsuleShape* clone = new (mem)b3CapsuleShape;
	*clone = *this;
	return clone;
}

b3AABB b3CapsuleShape::ComputeAABB() const
{
	b3Vec3 c1 = m_center1;
	b3Vec3 c2 = m_center2;
	b3Vec3 r(m_radius, m_radius, m_radius);
	
	b3AABB aabb;
	aabb.lowerBound = b3Min(c1, c2) - r;
	aabb.upperBound = b3Max(c1, c2) + r;
	return aabb;
}

bool b3CapsuleShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 Q = sphere.vertex;

	b3Vec3 A = m_center1;
	b3Vec3 B = m_center2;
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	scalar radius = m_radius + sphere.radius;

	if (v <= scalar(0))
	{
		// A
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	if (u <= scalar(0))
	{
		// B
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// AB
	scalar s = b3Dot(AB, AB);
	//B3_ASSERT(s > scalar(0));
	b3Vec3 P = (u * A + v * B) / s;

	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return false;
	}

	b3Vec3 AQ = Q - A;
	b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
	b3Vec3 n = b3Cross(AB_x_AQ, AB);
	if (b3Dot(n, AQ) < scalar(0))
	{
		n = -n;
	}
	n.Normalize();

	manifold->point = P;
	manifold->normal = n;
	return true;
}

void b3CapsuleShape::Draw(b3Draw* draw) const
{
	draw->DrawPoint(m_center1, scalar(4), b3Color_black);
	draw->DrawPoint(m_center2, scalar(4), b3Color_black);
	
	draw->DrawSegment(m_center1, m_center2, b3Color_black);
	
	draw->DrawSolidCapsule(b3Quat_identity, m_center1, m_center2, m_radius, b3Color_gray);
}
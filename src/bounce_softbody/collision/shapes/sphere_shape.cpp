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

#include <bounce_softbody/collision/shapes/sphere_shape.h>
#include <bounce_softbody/collision/geometry/sphere.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

b3SphereShape::b3SphereShape()
{
	m_type = e_sphere;
	m_radius = scalar(0);
	m_center.SetZero();
}

b3Shape* b3SphereShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3SphereShape));
	b3SphereShape* clone = new (mem) b3SphereShape;
	*clone = *this;
	return clone;
}

b3AABB b3SphereShape::ComputeAABB() const
{
	b3Vec3 center = m_center;
	b3Vec3 r(m_radius, m_radius, m_radius);
	
	b3AABB aabb;
	aabb.lowerBound = center - r;
	aabb.upperBound = center + r;
	return aabb;
}

bool b3SphereShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 center = m_center;
	scalar radius = m_radius + sphere.radius;
	scalar rr = radius * radius;
	b3Vec3 d = sphere.vertex - center;
	scalar dd = b3Dot(d, d);

	if (dd <= rr)
	{
		manifold->point = center;

		if (dd > B3_EPSILON * B3_EPSILON)
		{
			scalar distance = b3Sqrt(dd);
			
			manifold->normal = d / distance;
		}
		else
		{
			manifold->normal.Set(0, 1, 0);
		}

		return true;
	}

	return false;
}

void b3SphereShape::Draw(b3Draw* draw) const
{
	draw->DrawPoint(m_center, scalar(4), b3Color_black);
	draw->DrawSolidSphere(b3Quat_identity, m_center, m_radius, b3Color_gray);
}
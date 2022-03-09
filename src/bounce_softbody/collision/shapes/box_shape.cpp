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

#include <bounce_softbody/collision/shapes/box_shape.h>
#include <bounce_softbody/collision/geometry/sphere.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

b3BoxShape::b3BoxShape()
{
	m_type = e_box;
	m_radius = scalar(0);
	m_extents.Set(1, 1, 1);
	m_xf.SetIdentity();
}

b3Shape* b3BoxShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3BoxShape));
	b3BoxShape* clone = new (mem) b3BoxShape;
	*clone = *this;
	return clone;
}

b3AABB b3BoxShape::ComputeAABB() const
{
	b3Vec3 e = m_extents;

	b3Vec3 vertices[8] = 
	{
		b3Vec3(-e.x, -e.y, -e.z),
		b3Vec3(-e.x, -e.y,  e.z),
		b3Vec3(-e.x,  e.y, -e.z),
		b3Vec3(-e.x,  e.y,  e.z),
		b3Vec3(e.x, -e.y, -e.z),
		b3Vec3(e.x, -e.y,  e.z),
		b3Vec3(e.x,  e.y, -e.z),
		b3Vec3(e.x,  e.y,  e.z)
	};

	b3AABB aabb;
	aabb.Set(vertices, 8, m_xf);
	aabb.Extend(m_radius);

	return aabb;
}

bool b3BoxShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 e = m_extents;

	scalar radius = m_radius + sphere.radius;

	// Sphere center in the frame of the box.
	b3Vec3 cLocal = b3MulT(m_xf, sphere.vertex);

	// Box planes
	b3Plane planes[6] =
	{
		b3Plane(b3Vec3(scalar(1), scalar(0), scalar(0)), e.x),
		b3Plane(b3Vec3(scalar(-1), scalar(0), scalar(0)), e.x),
		b3Plane(b3Vec3(scalar(0), scalar(1), scalar(0)), e.y),
		b3Plane(b3Vec3(scalar(0), scalar(-1), scalar(0)), e.y),
		b3Plane(b3Vec3(scalar(0), scalar(0), scalar(1)), e.z),
		b3Plane(b3Vec3(scalar(0), scalar(0), scalar(-1)), e.z),
	};

	// Find the minimum separation face.	
	uint32 faceIndex = 0;
	scalar separation = -B3_MAX_SCALAR;

	for (uint32 i = 0; i < 6; ++i)
	{
		b3Plane plane = planes[i];
		scalar s = b3Distance(cLocal, plane);

		if (s > radius)
		{
			// Early out.
			return false;
		}

		if (s > separation)
		{
			faceIndex = i;
			separation = s;
		}
	}
	
	if (separation < scalar(0))
	{
		// The sphere center is inside the box.
		b3Plane pLocal = planes[faceIndex];

		b3Vec3 cBox = b3ClosestPointOnPlane(cLocal, pLocal);

		manifold->point = b3Mul(m_xf, cBox);
		manifold->normal = b3Mul(m_xf.rotation, pLocal.normal);

		return true;
	}

	// Closest point on box to sphere center
	b3Vec3 cBox = b3Clamp(cLocal, -e, e);
	
	b3Vec3 d = cLocal - cBox;

	scalar dd = b3LengthSquared(d);

	if (dd > radius * radius)
	{
		return false;
	}

	if (dd > B3_EPSILON * B3_EPSILON)
	{
		scalar distance = b3Sqrt(dd);
		b3Vec3 normal = d / distance;

		manifold->point = b3Mul(m_xf, cBox);
		manifold->normal = b3Mul(m_xf.rotation, normal);

		return true;
	}

	return false;
}

void b3BoxShape::Draw(b3Draw* draw) const
{
	b3Vec3 e = m_extents;

	b3Vec3 vertices[8] = 
	{
		b3Vec3(-e.x, -e.y, -e.z),
		b3Vec3(-e.x, -e.y,  e.z),
		b3Vec3(-e.x,  e.y, -e.z),
		b3Vec3(-e.x,  e.y,  e.z),
		b3Vec3(e.x, -e.y, -e.z),
		b3Vec3(e.x, -e.y,  e.z),
		b3Vec3(e.x,  e.y, -e.z),
		b3Vec3(e.x,  e.y,  e.z)
	};

	uint32 indices[36] =
	{
		0, 6, 4,
		0, 2, 6,
		0, 3, 2,
		0, 1, 3,
		2, 7, 6,
		2, 3, 7,
		4, 6, 7,
		4, 7, 5,
		0, 4, 5,
		0, 5, 1,
		1, 5, 7,
		1, 7, 3
	};

	for (uint32 i = 0; i < 36; i += 3)
	{
		b3Vec3 A = m_xf * vertices[indices[i]];
		b3Vec3 B = m_xf * vertices[indices[i + 1]];
		b3Vec3 C = m_xf * vertices[indices[i + 2]];
		b3Vec3 N = b3Normalize(b3Cross(B - A, C - A));

		draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
	}
}

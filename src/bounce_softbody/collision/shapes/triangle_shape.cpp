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

#include <bounce_softbody/collision/shapes/triangle_shape.h>
#include <bounce_softbody/collision/geometry/geometry.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

b3TriangleShape::b3TriangleShape()
{
	m_type = e_triangle;
	m_radius = scalar(0);
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

b3Shape* b3TriangleShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3TriangleShape));
	b3TriangleShape* clone = new (mem)b3TriangleShape;
	*clone = *this;
	return clone;
}

void b3TriangleShape::Set(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_vertex3 = v3;
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

b3AABB b3TriangleShape::ComputeAABB() const
{
	b3Vec3 lower = b3Min(m_vertex1, b3Min(m_vertex2, m_vertex3));
	b3Vec3 upper = b3Max(m_vertex1, b3Max(m_vertex2, m_vertex3));

	b3Vec3 r(m_radius, m_radius, m_radius);

	b3AABB aabb;
	aabb.lowerBound = lower - r;
	aabb.upperBound = upper + r;

	return aabb;
}

bool b3TriangleShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	// The sphere center.
	b3Vec3 Q = sphere.vertex;
	
	// ABC
	b3Vec3 A = m_vertex1, B = m_vertex2, C = m_vertex3;

	scalar radius = m_radius + sphere.radius;

	// Use the triangle normal if the closest point is exactly on the triangle plane.
	b3Vec3 triangleNormal = b3Cross(B - A, C - A);
	if (triangleNormal.Normalize() <= B3_EPSILON)
	{
		// TODO: Revise this to handle slivers.
		triangleNormal.Set(0, 1, 0);
	}

	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;
		
		return true;
	}

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		b3Vec3 P = C;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// Test edge regions		
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wABC[3] * wABC[2] <= scalar(0))
	{
		B3_ASSERT(wAB[2] > scalar(0));

		b3Vec3 P = (wAB[0] * A + wAB[1] * B) / wAB[2];
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		// Is there a face connected to AB?
		if (m_hasE1Vertex)
		{
			b3Vec3 A1 = m_e1Vertex;
			b3Vec3 B1 = B;
			b3Vec3 C1 = A;

			scalar wABC1[4];
			b3BarycentricCoordinates(wABC1, A1, B1, C1, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC1[0] > scalar(0) && wABC1[1] > scalar(0) && wABC1[2] > scalar(0))
			{
				return false;
			}
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && wABC[3] * wABC[0] <= scalar(0))
	{
		B3_ASSERT(wBC[2] > scalar(0));

		b3Vec3 P = (wBC[0] * B + wBC[1] * C) / wBC[2];
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		// Is there a face connected to AC?
		if (m_hasE2Vertex)
		{
			b3Vec3 A2 = m_e2Vertex;
			b3Vec3 B2 = C;
			b3Vec3 C2 = B;

			scalar wABC2[4];
			b3BarycentricCoordinates(wABC2, A2, B2, C2, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC2[0] > scalar(0) && wABC2[1] > scalar(0) && wABC2[2] > scalar(0))
			{
				return false;
			}
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && wABC[3] * wABC[1] <= scalar(0))
	{
		B3_ASSERT(wCA[2] > scalar(0));

		b3Vec3 P = (wCA[0] * C + wCA[1] * A) / wCA[2];
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		// Is there a face connected to CA?
		if (m_hasE3Vertex)
		{
			b3Vec3 A3 = m_e3Vertex;
			b3Vec3 B3 = A;
			b3Vec3 C3 = C;

			scalar wABC3[4];
			b3BarycentricCoordinates(wABC3, A3, B3, C3, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC3[0] > scalar(0) && wABC3[1] > scalar(0) && wABC3[2] > scalar(0))
			{
				return false;
			}
		}

		b3Vec3 n = triangleNormal;
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// R ABC/ACB
	B3_ASSERT(wABC[3] > scalar(0));

	b3Vec3 P = (wABC[0] * A + wABC[1] * B + wABC[2] * C) / wABC[3];
	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return false;
	}
	
	b3Vec3 n = triangleNormal;
	scalar len = b3Length(d);
	if (len > B3_EPSILON)
	{
		n = d / len;
	}

	manifold->point = P;
	manifold->normal = n;

	return true;
}

bool b3TriangleShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
{
	b3Vec3 p1 = input.p1;
	b3Vec3 p2 = input.p2;
	scalar maxFraction = input.maxFraction;

	b3Vec3 d = p2 - p1;

	if (b3LengthSquared(d) < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	b3Vec3 v1 = m_vertex1, v2 = m_vertex2, v3 = m_vertex3;
	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	
	if (b3LengthSquared(n) < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	n.Normalize();

	scalar num = b3Dot(n, v1 - p1);
	scalar den = b3Dot(n, d);

	if (den == scalar(0))
	{
		return false;
	}

	scalar t = num / den;

	// Is the intersection not on the segment?
	if (t < scalar(0) || maxFraction < t)
	{
		return false;
	}

	b3Vec3 Q = p1 + t * d;

	b3Vec3 A = v1, B = v2, C = v3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	// Barycentric coordinates for Q
	scalar u = b3Dot(QB_x_QC, AB_x_AC);
	scalar v = b3Dot(QC_x_QA, AB_x_AC);
	scalar w = b3Dot(QA_x_QB, AB_x_AC);

	// Is the intersection on the triangle?
	if (u >= scalar(0) && v >= scalar(0) && w >= scalar(0))
	{
		output->fraction = t;

		// Does the ray start from below or above the triangle?
		if (num > scalar(0))
		{
			output->normal = -n;
		}
		else
		{
			output->normal = n;
		}

		return true;
	}

	return false;
}

void b3TriangleShape::Draw(b3Draw* draw) const
{
	b3Vec3 A = m_vertex1, B = m_vertex2, C = m_vertex3;
	
	draw->DrawTriangle(A, B, C, b3Color_black);

	b3Vec3 N = b3Cross(B - A, C - A);
	N.Normalize();

	draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
	draw->DrawSolidTriangle(-N, A, C, B, b3Color_gray);
}
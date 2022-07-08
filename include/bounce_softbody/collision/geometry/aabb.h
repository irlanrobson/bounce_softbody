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

#ifndef B3_AABB_H
#define B3_AABB_H

#include <bounce_softbody/common/math/transform.h>
#include <bounce_softbody/collision/collision.h>
#include <bounce_softbody/collision/geometry/sphere.h>

// A min-max representation of a three-dimensional AABB.
struct b3AABB
{
	// Constructor. Does nothing for performance. 
	b3AABB() { }

	// Construct this AABB from center point and radius vector (half-extents).
	b3AABB(const b3Vec3& center, const b3Vec3& radius)
	{
		lowerBound = center - radius;
		upperBound = center + radius;
	}

	// Construct this AABB from center point and radius value (half-extent).
	b3AABB(const b3Vec3& center, scalar radius)
	{
		b3Vec3 r(radius, radius, radius);
		lowerBound = center - r;
		upperBound = center + r;
	}

	// Get relative position given a point. 
	// The point must be inside this AABB.
	// The closer the point is to lowerBound, closer the result is to (0, 0, 0).
	// The closer the point is to upperBound, closer the result is to (1, 1, 1).
	b3Vec3 GetRelativePosition(const b3Vec3& point) const
	{
		return (point - lowerBound) / GetDimensions();
	}

	// Get the support vertex in a given direction.
	b3Vec3 GetSupportVertex(const b3Vec3& direction) const
	{
		b3Vec3 support;
		support.x = direction.x < scalar(0) ? lowerBound.x : upperBound.x;
		support.y = direction.y < scalar(0) ? lowerBound.y : upperBound.y;
		support.z = direction.z < scalar(0) ? lowerBound.z : upperBound.z;
		return support;
	}

	// Compute this AABB from a list of points.
	void Compute(const b3Vec3* points, uint32 count)
	{
		lowerBound = upperBound = points[0];
		for (uint32 i = 1; i < count; ++i)
		{
			lowerBound = b3Min(lowerBound, points[i]);
			upperBound = b3Max(upperBound, points[i]);
		}
	}

	// Compute this AABB from a list of points and a transform.
	void Compute(const b3Vec3* points, uint32 count, const b3Transform& xf)
	{
		lowerBound = upperBound = b3Mul(xf, points[0]);
		for (uint32 i = 1; i < count; ++i)
		{
			b3Vec3 v = b3Mul(xf, points[i]);
			lowerBound = b3Min(lowerBound, v);
			upperBound = b3Max(upperBound, v);
		}
	}

	// Compute this AABB from a list of points, a local scale, and a transform.
	void Compute(const b3Vec3* points, uint32 count, const b3Vec3& scale, const b3Transform& xf)
	{
		lowerBound = upperBound = b3Mul(xf, b3Mul(scale, points[0]));
		for (uint32 i = 1; i < count; ++i)
		{
			b3Vec3 v = b3Mul(xf, b3Mul(scale, points[i]));
			lowerBound = b3Min(lowerBound, v);
			upperBound = b3Max(upperBound, v);
		}
	}
	
	// Set this AABB from a center point and a radius vector.
	void Set(const b3Vec3& center, const b3Vec3& r)
	{
		lowerBound = center - r;
		upperBound = center + r;
	}

	// Extend this AABB by a radius value.
	void Extend(scalar s) 
	{
		b3Vec3 r(s, s, s);
		lowerBound -= r;
		upperBound += r;
	}
	
	// Extend this AABB by a radius vector.
	void Extend(const b3Vec3& r)
	{
		lowerBound -= r;
		upperBound += r;
	}
	
	// Combine an AABB into this one.
	void Combine(const b3AABB& aabb)
	{
		lowerBound = b3Min(lowerBound, aabb.lowerBound);
		upperBound = b3Max(upperBound, aabb.upperBound);
	}

	// Combine two AABBs into this one.
	void Combine(const b3AABB& aabb1, const b3AABB& aabb2)
	{
		lowerBound = b3Min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = b3Max(aabb1.upperBound, aabb2.upperBound);
	}

	// Get the center of this AABB.
	b3Vec3 GetCenter() const 
	{
		return  scalar(0.5) * (lowerBound + upperBound);
	}

	// Get the half-extents of this AABB.
	b3Vec3 GetExtents() const
	{
		return scalar(0.5) * (upperBound - lowerBound);
	}

	// Get the width of this AABB.
	scalar GetWidth() const 
	{
		return upperBound.x - lowerBound.x;
	}

	// Get the height of this AABB.
	scalar GetHeight() const 
	{
		return upperBound.y - lowerBound.y;
	}

	// Get the depth of this AABB.
	scalar GetDepth() const 
	{
		return upperBound.z - lowerBound.z;
	}

	// Get the dimensions of this AABB.
	b3Vec3 GetDimensions() const
	{
		return b3Vec3(GetWidth(), GetHeight(), GetDepth());
	}

	// Get the volume of this AABB.
	scalar GetVolume() const 
	{
		b3Vec3 d = upperBound - lowerBound;
		return d.x * d.y * d.z;
	}

	// Get the surface area of this AABB.
	scalar GetSurfaceArea() const 
	{
		b3Vec3 d = upperBound - lowerBound;
		return scalar(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
	}

	// Get the index of the longest axis of this AABB.
	uint32 GetLongestAxis() const
	{
		b3Vec3 d = upperBound - lowerBound;
		
		scalar maxValue = d.x;
		uint32 maxIndex = 0;
		
		if (d.y > maxValue)
		{
			maxValue = d.y;
			maxIndex = 1;
		}
		
		if (d.z > maxValue)
		{
			maxValue = d.z;
			maxIndex = 2;
		}
		
		return maxIndex;
	}

	// Test if this AABB contains a point.
	bool Contains(const b3Vec3& point) const
	{
		return	lowerBound.x <= point.x && point.x <= upperBound.x &&
				lowerBound.y <= point.y && point.y <= upperBound.y &&
				lowerBound.z <= point.z && point.z <= upperBound.z;
	}

	// Test if this AABB contains another AABB.
	bool Contains(const b3AABB& aabb) const
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && lowerBound.z <= aabb.lowerBound.z;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		result = result && aabb.upperBound.z <= upperBound.z;
		return result;
	}

	// From Real-time Collision Detection, p179.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
	{
		scalar tmin = -B3_MAX_SCALAR;
		scalar tmax = B3_MAX_SCALAR;

		b3Vec3 p = input.p1;
		b3Vec3 d = input.p2 - input.p1;
		b3Vec3 absD = b3Abs(d);

		b3Vec3 normal;
		normal.SetZero();

		for (uint32 i = 0; i < 3; ++i)
		{
			if (absD[i] < B3_EPSILON)
			{
				// Parallel.
				if (p[i] < lowerBound[i] || upperBound[i] < p[i])
				{
					return false;
				}
			}
			else
			{
				scalar inv_d = scalar(1) / d[i];
				scalar t1 = (lowerBound[i] - p[i]) * inv_d;
				scalar t2 = (upperBound[i] - p[i]) * inv_d;

				// Sign of the normal vector.
				scalar s = scalar(-1);

				if (t1 > t2)
				{
					b3Swap(t1, t2);
					s = scalar(1);
				}

				// Push the min up
				// tmin = max(tmin, t1) as in errata.
				if (t1 > tmin)
				{
					normal.SetZero();
					normal[i] = s;
					tmin = t1;
				}

				// Pull the max down
				// tmax = min(tmax, t2) as in errata.
				tmax = b3Min(tmax, t2);

				if (tmin > tmax)
				{
					return false;
				}
			}
		}

		// Does the ray start inside the box?
		// Does the ray intersect beyond the max fraction?
		if (tmin < scalar(0) || input.maxFraction < tmin)
		{
			return false;
		}

		// Intersection.
		output->fraction = tmin;
		output->normal = normal;
		return true;
	}

	// Translate this AABB.
	void Translate(const b3Vec3& translation)
	{
		lowerBound += translation;
		upperBound += translation;
	}

	// Scale this AABB by a scale.
	// The scale can be non-uniform and negative.
	void Scale(const b3Vec3& scale)
	{
		b3Vec3 scaledLower = b3Mul(scale, lowerBound);
		b3Vec3 scaledUpper = b3Mul(scale, upperBound);
		
		lowerBound = b3Min(scaledLower, scaledUpper);
		upperBound = b3Max(scaledLower, scaledUpper);
	}

	// Transform this AABB.
	void Transform(const b3Transform& xf)
	{
		b3Vec3 localCenter = GetCenter();
		b3Vec3 localRadius = GetExtents();

		b3Vec3 center = b3Mul(xf, localCenter);

		b3Mat33 absRotation = b3Abs(xf.rotation.GetRotationMatrix());

		b3Vec3 radius;
		radius.x = b3Dot(localRadius, absRotation.x);
		radius.y = b3Dot(localRadius, absRotation.y);
		radius.z = b3Dot(localRadius, absRotation.z);

		Set(center, radius);
	}

	// Transform this AABB by a transform and a local scale.
	void Transform(const b3Transform& xf, const b3Vec3& scale)
	{
		b3Vec3 localCenter = GetCenter();
		b3Vec3 localRadius = b3Mul(b3Abs(scale), GetExtents());

		b3Vec3 center = b3Mul(xf, localCenter);

		b3Mat33 absRotation = b3Abs(xf.rotation.GetRotationMatrix());

		b3Vec3 radius;
		radius.x = b3Dot(localRadius, absRotation.x);
		radius.y = b3Dot(localRadius, absRotation.y);
		radius.z = b3Dot(localRadius, absRotation.z);

		Set(center, radius);
	}
	
	b3Vec3 lowerBound; // lower vertex
	b3Vec3 upperBound; // upper vertex
};

// Compute an AABB that encloses two AABBs.
inline b3AABB b3Combine(const b3AABB& a, const b3AABB& b) 
{
	b3AABB aabb;
	aabb.lowerBound = b3Min(a.lowerBound, b.lowerBound);
	aabb.upperBound = b3Max(a.upperBound, b.upperBound);
	return aabb;
}

// Test if two AABBs are overlapping.
// From Real-time Collision Detection, p79.
inline bool b3TestOverlap(const b3AABB& a, const b3AABB& b)
{
	// Exit with no intersection if separated along an axis.
	if (a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x) 
	{
		return false;
	}
	
	if (a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y)
	{
		return false;
	}
	
	if (a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z)
	{
		return false;
	}

	// Overlapping on all axes means AABBs are intersecting.
	return true;
}

// Return the closest point on AABB to a given point.
inline b3Vec3 b3ClosestPointOnAABB(const b3AABB& aabb, const b3Vec3& point)
{
	return b3Clamp(point, aabb.lowerBound, aabb.upperBound);
}

// Test if an AABB and a sphere are overlapping.
inline bool b3TestOverlap(const b3AABB& aabb, const b3Sphere& sphere)
{
	b3Vec3 point = b3ClosestPointOnAABB(aabb, sphere.vertex);
	return b3DistanceSquared(sphere.vertex, point) <= sphere.radius * sphere.radius;
}

#endif
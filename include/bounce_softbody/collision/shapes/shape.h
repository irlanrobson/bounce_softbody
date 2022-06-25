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

#ifndef B3_SHAPE_H
#define B3_SHAPE_H

#include <bounce_softbody/collision/geometry/aabb.h>

struct b3Sphere;

class b3Draw;
class b3BlockAllocator;

// Sphere contact manifold. 
// c2 = center
// separation = dot(c2 - c1, normal) - r1 - r2
struct b3SphereManifold
{
	b3Vec3 point; // contact point on the shape
	b3Vec3 normal; // contact normal on the shape towards the sphere
};

// Collision shape in static environment used for collision detection.
class b3Shape
{
public:
	enum Type
	{
		e_sphere = 0,
		e_capsule = 1,
		e_triangle = 2,
		e_box = 3,
		e_mesh = 4,
		e_typeCount = 5
	};

	// Default dtor.
	virtual ~b3Shape() { }
	
	// Return the shape type.
	Type GetType() const;

	// Clone the concrete shape using the provided allocator.
	virtual b3Shape* Clone(b3BlockAllocator* allocator) const = 0;

	// Compute the AABB for this shape.
	virtual b3AABB ComputeAABB() const = 0;

	// Generate the contact manifold for a given sphere.
	// Return true if the given sphere is colliding with this shape, false otherwise.
	virtual bool CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const = 0;

	// Debug draw this shape.
	virtual void Draw(b3Draw* draw) const = 0;

	// Shape type.
	Type m_type;

	// Shape radius.
	scalar m_radius;
};

inline b3Shape::Type b3Shape::GetType() const
{
	return m_type;
}

#endif

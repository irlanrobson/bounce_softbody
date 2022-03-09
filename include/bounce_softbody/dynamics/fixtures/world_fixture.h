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

#ifndef B3_WORLD_FIXTURE_H
#define B3_WORLD_FIXTURE_H

#include <bounce_softbody/collision/shapes/shape.h>

class b3Draw;
class b3Body;

// World fixture definition.
// The given shape will be cloned and can be a temporary object.
struct b3WorldFixtureDef
{
	b3WorldFixtureDef()
	{
		shape = nullptr;
		friction = scalar(0.5);
	}

	// Shape to be cloned.
	b3Shape* shape;
	
	// Coefficient of friction in the range [0, 1].
	scalar friction;
};

// World fixture. This encapsulates a collision shape in the static environment.
class b3WorldFixture
{
public:
	// Return the type of the child shape.
	b3Shape::Type GetType() const;
	
	// Return the child shape.
	b3Shape* GetShape();
	const b3Shape* GetShape() const;

	// Compute an AABB for the shape.
	b3AABB ComputeAABB() const;

	// Generate the contact manifold for a given sphere.
	// Return true if the given sphere is colliding with the child shape, false otherwise.
	bool CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const;

	// Draw the child shape.
	void Draw(b3Draw* draw) const;

	// Set the coefficient of friction.
	// This represents both static and dynamic friction.
	void SetFriction(scalar friction);

	// Get the coefficient of friction.
	scalar GetFriction() const;

	// Return the next world fixture in the body list of world fixtures.
	b3WorldFixture* GetNext() { return m_next; }
	const b3WorldFixture* GetNext() const { return m_next; }
private:
	friend class b3Body;
	friend class b3ContactManager;
	friend class b3SphereAndShapeContact;
	friend class b3FrictionSolver;

	// Ctor.
	b3WorldFixture();
	
	// Create/destroy this shape. 
	void Create(b3BlockAllocator* allocator, b3Body* body, const b3WorldFixtureDef& def);
	void Destroy(b3BlockAllocator* allocator);

	// Destroy contacts.
	void DestroyContacts();

	// The collision shape.
	b3Shape* m_shape;

	// Coefficient of friction.
	scalar m_friction;

	// Body.
	b3Body* m_body;

	// Body list links.
	b3WorldFixture* m_prev;
	b3WorldFixture* m_next;
};

inline b3Shape::Type b3WorldFixture::GetType() const
{
	return m_shape->m_type;
}

inline b3Shape* b3WorldFixture::GetShape()
{
	return m_shape;
}

inline const b3Shape* b3WorldFixture::GetShape() const
{
	return m_shape;
}

inline b3AABB b3WorldFixture::ComputeAABB() const
{
	return m_shape->ComputeAABB();
}

inline bool b3WorldFixture::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	return m_shape->CollideSphere(manifold, sphere);
}

inline void b3WorldFixture::Draw(b3Draw* draw) const
{
	m_shape->Draw(draw);
}

inline void b3WorldFixture::SetFriction(scalar friction)
{
	B3_ASSERT(friction >= scalar(0));
	m_friction = friction;
}

inline scalar b3WorldFixture::GetFriction() const
{
	return m_friction;
}

#endif

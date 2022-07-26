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

#ifndef B3_FIXTURE_H
#define B3_FIXTURE_H

#include <bounce_softbody/common/math/math.h>

class b3Body;

enum b3FixtureType
{
	e_sphereFixture,
	e_triangleFixture,
	e_tetrahedronFixture
};

// Fixture definition.
struct b3FixtureDef
{
	b3FixtureDef()
	{
		radius = scalar(0);
		density = scalar(0);
		friction = scalar(0);
		userIndex = B3_MAX_U32;
	}

	// Shape type.
	b3FixtureType type;
	
	// Shape radius.
	scalar radius;

	// Density. Set to zero to disable mass contribution.
	scalar density;

	// Coefficient of friction.
	scalar friction;

	// User index pointing to anything.
	uint32 userIndex;
};

// This is an internal body fixture.
class b3Fixture
{
public:
	// Get the shape type.
	b3FixtureType GetType() const;
	
	// Set the shape radius.
	void SetRadius(scalar radius);

	// Get the shape radius.
	scalar GetRadius() const;

	// Set the fixture density. This will not automatically adjust the mass 
	// of the particles. Set this to zero to disable fixture mass contribution.
	void SetDensity(scalar density);

	// Get the fixture density.
	scalar GetDensity() const;

	// Set the coefficient of friction. 
	// This represents both static and dynamic friction.
	void SetFriction(scalar friction);

	// Get the coefficient of friction.
	scalar GetFriction() const;

	// Set the user index.
	void SetUserIndex(uint32 userIndex);

	// Get the user index.
	uint32 GetUserIndex() const;

	// Get the body.
	b3Body* GetBody();
	const b3Body* GetBody() const;
protected:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ContactManager;

	b3Fixture(const b3FixtureDef& def, b3Body* body);
	virtual ~b3Fixture() { }

	// Type
	b3FixtureType m_type;

	// Radius
	scalar m_radius;

	// Density
	scalar m_density;

	// Coefficient of friction
	scalar m_friction;

	// User index
	uint32 m_userIndex;

	// Parent body
	b3Body* m_body;
};

inline b3Fixture::b3Fixture(const b3FixtureDef& def, b3Body* body)
{
	m_radius = def.radius;
	m_density = def.density;
	m_friction = def.friction;
	m_userIndex = def.userIndex;
	m_body = body;
}

inline b3FixtureType b3Fixture::GetType() const
{
	return m_type;
}

inline void b3Fixture::SetRadius(scalar radius)
{
	B3_ASSERT(radius >= scalar(0));
	m_radius = radius;
}

inline scalar b3Fixture::GetRadius() const
{
	return m_radius;
}

inline void b3Fixture::SetDensity(scalar density)
{
	B3_ASSERT(density >= scalar(0));
	m_density = density;
}

inline scalar b3Fixture::GetDensity() const
{
	return m_density;
}

inline void b3Fixture::SetFriction(scalar friction)
{
	B3_ASSERT(friction >= scalar(0));
	m_friction = friction;
}

inline scalar b3Fixture::GetFriction() const
{
	return m_friction;
}

inline void b3Fixture::SetUserIndex(uint32 userIndex)
{
	m_userIndex = userIndex;
}

inline uint32 b3Fixture::GetUserIndex() const
{
	return m_userIndex;
}

inline b3Body* b3Fixture::GetBody()
{
	return m_body;
}

inline const b3Body* b3Fixture::GetBody() const
{
	return m_body;
}

#endif
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

#ifndef B3_SPHERE_FIXTURE_H
#define B3_SPHERE_FIXTURE_H

#include <bounce_softbody/dynamics/fixtures/fixture.h>
#include <bounce_softbody/collision/geometry/aabb.h>

class b3Particle;

// Sphere fixture definition.
struct b3SphereFixtureDef : public b3FixtureDef
{
	b3SphereFixtureDef()
	{
		type = e_sphereFixture;
	}

	// Particle
	b3Particle* p = nullptr;
};

// Sphere fixture. This is used for collision detection and resolution.
class b3SphereFixture : public b3Fixture
{
public:
	// Compute AABB
	b3AABB ComputeAABB() const;
	
	// Return the particle.
	b3Particle* GetParticle() { return m_p; }
	const b3Particle* GetParticle() const { return m_p; }

	// Return the next sphere in the body.
	b3SphereFixture* GetNext() { return m_next; };
	const b3SphereFixture* GetNext() const { return m_next; };
private:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ContactManager;
	friend class b3SphereAndShapeContact;
	friend class b3BodySolver;
	friend class b3FrictionSolver;
	
	b3SphereFixture(const b3SphereFixtureDef& def, b3Body* body);
	
	// Destroy contacts
	void DestroyContacts();

	// Particle
	b3Particle* m_p;

	// Links to the body list.
	b3SphereFixture* m_prev;
	b3SphereFixture* m_next;
};

#endif
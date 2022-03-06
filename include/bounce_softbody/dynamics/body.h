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

#ifndef B3_BODY_H
#define B3_BODY_H

#include <bounce_softbody/common/memory/stack_allocator.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/template/list.h>
#include <bounce_softbody/collision/trees/dynamic_tree.h>
#include <bounce_softbody/dynamics/contact_manager.h>

class b3Draw;

struct b3ParticleDef;
class b3Particle;

struct b3ForceDef;
class b3Force;

struct b3SphereFixtureDef;
class b3SphereFixture;

struct b3TriangleFixtureDef;
class b3TriangleFixture;

struct b3TetrahedronFixtureDef;
class b3TetrahedronFixture;

struct b3WorldFixtureDef;
class b3WorldFixture;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3TimeStep;

struct b3BodyRayCastSingleOutput
{
	b3TriangleFixture* triangle;
	scalar fraction;
	b3Vec3 normal;
};

// A body represents a deformable body as a collection of particles.
// Particles may be connected with each other by forces.
class b3Body
{
public:
	b3Body();
	virtual ~b3Body();

	// Create a particle.
	b3Particle* CreateParticle(const b3ParticleDef& def);

	// Destroy a given particle.
	void DestroyParticle(b3Particle* particle);

	// Return the list of particles in this body.
	const b3List<b3Particle>& GetParticleList() const;
	b3List<b3Particle>& GetParticleList();

	// Create a force.
	b3Force* CreateForce(const b3ForceDef& def);

	// Destroy a given force.
	void DestroyForce(b3Force* force);

	// Return the list of forces in this body.
	const b3List<b3Force>& GetForceList() const;
	b3List<b3Force>& GetForceList();

	// Create a sphere fixture.
	b3SphereFixture* CreateSphere(const b3SphereFixtureDef& def);

	// Destroy a given sphere fixture.
	void DestroySphere(b3SphereFixture* fixture);
	
	// Return the list of spheres in this body.
	const b3List<b3SphereFixture>& GetSphereList() const;
	b3List<b3SphereFixture>& GetSphereList();

	// Create a triangle fixture.
	b3TriangleFixture* CreateTriangle(const b3TriangleFixtureDef& def);

	// Destroy a given triangle fixture.
	void DestroyTriangle(b3TriangleFixture* fixture);

	// Return the list of triangles in this body.
	const b3List<b3TriangleFixture>& GetTriangleList() const;
	b3List<b3TriangleFixture>& GetTriangleList();

	// Create a tetrahedron fixture.
	b3TetrahedronFixture* CreateTetrahedron(const b3TetrahedronFixtureDef& def);

	// Destroy a given tetrahedron fixture.
	void DestroyTetrahedron(b3TetrahedronFixture* fixture);

	// Return the list of tetrahedrons in this body.
	const b3List<b3TetrahedronFixture>& GetTetrahedronList() const;
	b3List<b3TetrahedronFixture>& GetTetrahedronList();

	// Create a new world fixture.
	b3WorldFixture* CreateFixture(const b3WorldFixtureDef& def);

	// Destroy a given world fixture.
	void DestroyFixture(b3WorldFixture* fixture);

	// Return the list of world fixtures in this body.
	const b3List<b3WorldFixture>& GetFixtureList() const;
	b3List<b3WorldFixture>& GetFixtureList();

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Perform a time step given the number of force solver and subsolver iterations. 
	// Use one force iteration for reasonable performance. 
	void Step(scalar dt, u32 forceIterations, u32 forceSubIterations);

	// Perform a ray cast with the body.
	bool RayCastSingle(b3BodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Return the kinetic energy in this system.
	scalar GetEnergy() const;

	// Debug draw the body entities.
	void Draw(b3Draw* draw) const;
protected:
	friend class b3Particle;
	friend class b3SphereFixture;
	friend class b3TriangleFixture;
	friend class b3TetrahedronFixture;
	friend class b3WorldFixture;
	friend class b3ContactManager;
	
	// Rest the mass data of the body.
	void ResetMass();

	// Solve
	void Solve(const b3TimeStep& step);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Block allocator
	b3BlockAllocator m_blockAllocator;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// List of particles
	b3List<b3Particle> m_particleList;

	// List of forces
	b3List<b3Force> m_forceList;

	// List of spheres
	b3List<b3SphereFixture> m_sphereList;
	
	// List of triangles
	b3List<b3TriangleFixture> m_triangleList;
	
	// List of tetrahedrons
	b3List<b3TetrahedronFixture> m_tetrahedronList;
	
	// List of world fixtures
	b3List<b3WorldFixture> m_fixtureList;

	// Contact manager
	b3ContactManager m_contactManager;

	// Dynamic tree.
	b3DynamicTree m_tree;
};

inline void b3Body::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3Body::GetGravity() const
{
	return m_gravity;
}

inline const b3List<b3Force>& b3Body::GetForceList() const
{
	return m_forceList;
}

inline b3List<b3Force>& b3Body::GetForceList() 
{
	return m_forceList;
}

inline const b3List<b3Particle>& b3Body::GetParticleList() const
{
	return m_particleList;
}

inline b3List<b3Particle>& b3Body::GetParticleList() 
{
	return m_particleList;
}

inline const b3List<b3SphereFixture>& b3Body::GetSphereList() const
{
	return m_sphereList;
}

inline b3List<b3SphereFixture>& b3Body::GetSphereList() 
{
	return m_sphereList;
}

inline const b3List<b3TriangleFixture>& b3Body::GetTriangleList() const
{
	return m_triangleList;
}

inline b3List<b3TriangleFixture>& b3Body::GetTriangleList() 
{
	return m_triangleList;
}

inline const b3List<b3TetrahedronFixture>& b3Body::GetTetrahedronList() const
{
	return m_tetrahedronList;
}

inline b3List<b3TetrahedronFixture>& b3Body::GetTetrahedronList() 
{
	return m_tetrahedronList;
}

inline const b3List<b3WorldFixture>& b3Body::GetFixtureList() const
{
	return m_fixtureList;
}

inline b3List<b3WorldFixture>& b3Body::GetFixtureList() 
{
	return m_fixtureList;
}

#endif
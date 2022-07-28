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

#ifndef B3_FORCE_H
#define B3_FORCE_H

#include <bounce_softbody/common/settings.h>

class b3BlockAllocator;
class b3Particle;

struct b3SparseForceSolverData;

// Force types
enum b3ForceType
{
	e_unknownForce,
	e_stretchForce,
	e_shearForce,
	e_springForce,
	e_mouseForce,
	e_triangleElementForce,
	e_tetrahedronElementForce,
};

// Force definition.
struct b3ForceDef
{
	b3ForceDef()
	{
		type = e_unknownForce;
		userIndex = B3_MAX_U32;
	}
	
	// Force type.
	b3ForceType type;

	// User index.
	uint32 userIndex;
};

// Forces acting on a set of particles.
class b3Force
{
public:
	// Get the force type.
	b3ForceType GetType() const;

	// Does this force contain a given particle?
	virtual bool Contains(const b3Particle* particle) const = 0;

	// Get the user index.
	uint32 GetUserIndex() const { return m_userIndex; }

	// Get the next force in the body force list.
	const b3Force* GetNext() const;
	b3Force* GetNext();
protected:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ForceSolver;
	friend class b3ForceModel;

	// Factory create and destroy.
	static b3Force* Create(const b3ForceDef* def, b3BlockAllocator* allocator);
	static void Destroy(b3Force* force, b3BlockAllocator* allocator);

	b3Force();
	virtual ~b3Force() { }

	// Clear internal forces stored for the user.
	virtual void ClearForces() = 0;

	// Apply forces and Jacobians.
	virtual void ApplyForces(const b3SparseForceSolverData* data) = 0;

	// Force type.
	b3ForceType m_type;
	
	// User index.
	uint32 m_userIndex;

	// Links to body list.
	b3Force* m_prev;
	b3Force* m_next;
};

inline b3ForceType b3Force::GetType() const
{
	return m_type;
}

inline const b3Force* b3Force::GetNext() const
{
	return m_next;
}

inline b3Force* b3Force::GetNext()
{
	return m_next;
}

#endif
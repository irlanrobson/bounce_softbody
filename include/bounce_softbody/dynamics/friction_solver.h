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

#ifndef B3_FRICTION_SOLVER_H
#define B3_FRICTION_SOLVER_H

#include <bounce_softbody/common/math/math.h>
#include <bounce_softbody/dynamics/time_step.h>

class b3StackAllocator;
class b3Particle;
class b3SphereAndShapeContact;

struct b3FrictionSolverDef
{
	b3TimeStep step;
	uint32 shapeContactCount;
	b3SphereAndShapeContact** shapeContacts;
};

// Mixed friction law.
inline scalar b3MixFriction(scalar u1, scalar u2)
{
	return b3Sqrt(u1 * u2);
}

class b3FrictionSolver
{
public:
	b3FrictionSolver(const b3FrictionSolverDef& def);
	
	void Solve();
protected:
	b3TimeStep m_step;
	b3StackAllocator* m_allocator;
	uint32 m_shapeContactCount;
	b3SphereAndShapeContact** m_shapeContacts;
};

#endif
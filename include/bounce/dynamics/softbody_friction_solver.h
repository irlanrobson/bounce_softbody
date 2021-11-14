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

#ifndef B3_SOFTBODY_FRICTION_SOLVER_H
#define B3_SOFTBODY_FRICTION_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/dynamics/softbody_time_step.h>

class b3StackAllocator;
class b3SoftBodyParticle;
class b3SoftBodySphereAndShapeContact;

struct b3SoftBodyFrictionSolverDef
{
	b3SoftBodyTimeStep step;
	u32 shapeContactCount;
	b3SoftBodySphereAndShapeContact** shapeContacts;
};

// Mixed friction law.
inline scalar b3MixFriction(scalar u1, scalar u2)
{
	return b3Sqrt(u1 * u2);
}

class b3SoftBodyFrictionSolver
{
public:
	b3SoftBodyFrictionSolver(const b3SoftBodyFrictionSolverDef& def);
	
	void Solve();
protected:
	b3SoftBodyTimeStep m_step;
	b3StackAllocator* m_allocator;
	u32 m_shapeContactCount;
	b3SoftBodySphereAndShapeContact** m_shapeContacts;
};

#endif
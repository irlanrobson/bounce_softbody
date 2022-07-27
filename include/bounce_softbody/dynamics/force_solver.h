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

#ifndef B3_FORCE_SOLVER_H
#define B3_FORCE_SOLVER_H

#include <bounce_softbody/dynamics/time_step.h>
#include <bounce_softbody/common/math/vec3.h>

class b3StackAllocator;
class b3Particle;
class b3Force;
class b3Contact;

struct b3ForceSolverDef
{
	b3TimeStep step;
	b3StackAllocator* allocator;
	uint32 particleCount;
	b3Particle** particles;
	uint32 forceCount;
	b3Force** forces;
	uint32 contactCount;
	b3Contact** contacts;
};

class b3ForceSolver
{
public:
	b3ForceSolver(const b3ForceSolverDef& def);
	~b3ForceSolver();

	void Solve(const b3Vec3& gravity);
private:
	b3TimeStep m_step;

	b3StackAllocator* m_allocator;

	uint32 m_particleCount;
	b3Particle** m_particles;

	uint32 m_forceCount;
	b3Force** m_forces;

	uint32 m_contactCount;
	b3Contact** m_contacts;
};

#endif
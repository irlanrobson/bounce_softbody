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

#ifndef B3_CONTACT_H
#define B3_CONTACT_H

#include <bounce_softbody/common/math/vec3.h>

struct b3SparseForceSolverData;
struct b3TimeStep;

// Mixed friction law.
inline scalar b3MixFriction(scalar u1, scalar u2)
{
	return b3Sqrt(u1 * u2);
}

// A contact acting on a set of particles.
class b3Contact
{
public:
	// Default dtor.
	virtual ~b3Contact() { }

	// Compute force solver data.
	virtual void ApplyForces(const b3SparseForceSolverData* data) = 0;

	// Solve friction constraints.
	virtual void ApplyFriction(const b3TimeStep& step, const b3Vec3& gravity) = 0;
};

#endif
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

#ifndef B3_SPHERE_AND_SHAPE_CONTACT_H
#define B3_SPHERE_AND_SHAPE_CONTACT_H

#include <bounce_softbody/common/math/vec3.h>

class b3BlockAllocator;
class b3SphereFixture;
class b3WorldFixture;

struct b3SparseForceSolverData;

// A contact between a sphere and a shape.
class b3SphereAndShapeContact
{
public:
	static b3SphereAndShapeContact* Create(b3SphereFixture* fixture1, b3WorldFixture* fixture2, b3BlockAllocator* allocator);
	static void Destroy(b3SphereAndShapeContact* contact, b3BlockAllocator* allocator);

	b3SphereAndShapeContact(b3SphereFixture* fixture1, b3WorldFixture* fixture2);

	void Update();
	
	void ComputeForces(const b3SparseForceSolverData* data);

	b3SphereFixture* m_f1;
	b3WorldFixture* m_f2;
	bool m_active;
	b3Vec3 m_tangent1, m_tangent2;
	scalar m_normalForce;
	b3SphereAndShapeContact* m_prev;
	b3SphereAndShapeContact* m_next;
};

#endif
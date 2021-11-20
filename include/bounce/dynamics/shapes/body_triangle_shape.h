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

#ifndef B3_BODY_TRIANGLE_SHAPE_H
#define B3_BODY_TRIANGLE_SHAPE_H

#include <bounce/dynamics/shapes/body_shape.h>
#include <bounce/collision/geometry/aabb.h>
#include <bounce/common/template/list.h>

struct b3RayCastInput;
struct b3RayCastOutput;

class b3Particle;

// Body triangle shape definition.
struct b3BodyTriangleShapeDef : public b3BodyShapeDef
{
	b3BodyTriangleShapeDef()
	{
		type = e_bodyTriangleShape;
	}

	// Particles
	b3Particle* p1;
	b3Particle* p2;
	b3Particle* p3;

	// Rest vertices
	b3Vec3 v1, v2, v3;
};

// Triangle shape.
class b3BodyTriangleShape : public b3BodyShape
{
public:
	// Return the particle 1.
	b3Particle* GetParticle1() { return m_p1; }
	const b3Particle* GetParticle1() const { return m_p1; }

	// Return the particle 2.
	b3Particle* GetParticle2() { return m_p2; }
	const b3Particle* GetParticle2() const { return m_p2; }

	// Return the particle 3.
	b3Particle* GetParticle3() { return m_p3; }
	const b3Particle* GetParticle3() const { return m_p3; }

	// Return the next triangle shape in the body list of shapes.
	b3BodyTriangleShape* GetNext() { return m_next; }
	const b3BodyTriangleShape* GetNext() const { return m_next; }

	// Ray cast against this triangle.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const;
private:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ContactManager;
	friend class b3List<b3BodyTriangleShape>;

	b3BodyTriangleShape(const b3BodyTriangleShapeDef& def, b3Body* body);
	
	// Compute AABB
	b3AABB ComputeAABB() const;

	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Particles
	b3Particle* m_p1;
	b3Particle* m_p2;
	b3Particle* m_p3;

	// Rest area. Used for computing the mass of the particles.
	scalar m_area;

	// Dynamic tree proxy.
	u32 m_proxyId;

	// Links to the body triangle shape list.
	b3BodyTriangleShape* m_prev;
	b3BodyTriangleShape* m_next;
};

#endif
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

#ifndef B3_MOUSE_FORCE_H
#define B3_MOUSE_FORCE_H

#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/common/math/vec3.h>

// Mouse force definition.
// This requires defining a particle and a triangle 
// and the coordinates of the particle local to the triangle 
// in the rest state using a barycentric coordinate system.
// You must also provide spring parameters.
struct b3MouseForceDef : public b3ForceDef
{
	b3MouseForceDef()
	{
		type = e_mouseForce; 
		w2 = scalar(0);
		w3 = scalar(0);
		w4 = scalar(0);
		length = scalar(0);
		stiffness = scalar(0);
		dampingStiffness = scalar(0);
	}

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;
	
	// Particle 3
	b3Particle* p3;
	
	// Particle 4
	b3Particle* p4;
	
	// Reference barycentric coordinates for p1 on triangle (p2, p3, p4)
	scalar w2, w3, w4;

	// Natural spring length
	scalar length;

	// Stiffness
	scalar stiffness;

	// Damping stiffness
	scalar dampingStiffness;
};

// Mouse force acting on a particle and triangle.
// This force will keep a point on one particle and the other on the 
// triangle to a given desired distance.
class b3MouseForce : public b3Force
{
public:
	// Does this force contain a given particle?
	bool Contains(const b3Particle* particle) const;

	// Get the particle 1.
	const b3Particle* GetParticle1() const { return m_p1; }
	b3Particle* GetParticle1() { return m_p1; }

	// Get the particle 2.
	const b3Particle* GetParticle2() const { return m_p2; }
	b3Particle* GetParticle2() { return m_p2; }
	
	// Get the particle 3.
	const b3Particle* GetParticle3() const { return m_p3; }
	b3Particle* GetParticle3() { return m_p3; }
	
	// Get the particle 4.
	const b3Particle* GetParticle4() const { return m_p4; }
	b3Particle* GetParticle4() { return m_p4; }
	
	// Set the natural spring length.
	void SetLength(scalar restLength);

	// Get the natural spring length.
	scalar GetLength() const;

	// Set the spring stiffness.
	void SetStiffness(scalar stiffness);

	// Get the spring stiffness.
	scalar GetStiffness() const;

	// Set the damping stiffness.
	void SetDampingStiffness(scalar dampingStiffness);

	// Get the damping stiffness.
	scalar GetDampingStiffness() const;

	// Get the force acting on particle 1.
	b3Vec3 GetActionForce1() const;

	// Get the force acting on particle 2.
	b3Vec3 GetActionForce2() const;

	// Get the force acting on particle 3.
	b3Vec3 GetActionForce3() const;

	// Get the force acting on particle 4.
	b3Vec3 GetActionForce4() const;
private:
	friend class b3Force;

	b3MouseForce(const b3MouseForceDef* def);
	
	void ClearForces();
	void ApplyForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3Particle* m_p1;

	// Particle 2
	b3Particle* m_p2;
	
	// Particle 3
	b3Particle* m_p3;
	
	// Particle 4
	b3Particle* m_p4;
	
	// Barycentric coordinates in the rest state
	scalar m_w2, m_w3, m_w4;

	// Spring stiffness
	scalar m_ks;

	// Damping stiffness
	scalar m_kd;

	// Rest length
	scalar m_L0;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3, m_f4;
};

inline void b3MouseForce::SetLength(scalar length)
{
	B3_ASSERT(length >= scalar(0));
	m_L0 = length;
}

inline scalar b3MouseForce::GetLength() const
{
	return m_L0;
}

inline void b3MouseForce::SetStiffness(scalar stiffness)
{
	B3_ASSERT(stiffness >= scalar(0));
	m_ks = stiffness;
}

inline scalar b3MouseForce::GetStiffness() const
{
	return m_ks;
}

inline void b3MouseForce::SetDampingStiffness(scalar dampingStiffness)
{
	B3_ASSERT(dampingStiffness >= scalar(0));
	m_kd = dampingStiffness;
}

inline scalar b3MouseForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3MouseForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3MouseForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3MouseForce::GetActionForce3() const
{
	return m_f3;
}

inline b3Vec3 b3MouseForce::GetActionForce4() const
{
	return m_f4;
}

#endif
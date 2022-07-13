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

#ifndef B3_SPRING_FORCE_H
#define B3_SPRING_FORCE_H

#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/common/math/vec3.h>

// Spring force definition.
// This requires defining two particles, the 
// natural spring rest length, and the spring parameters.
struct b3SpringForceDef : public b3ForceDef
{
	b3SpringForceDef()
	{
		type = e_springForce;
		length = scalar(0);
		stiffness = scalar(0);
		dampingStiffness = scalar(0);
	}

	// Initialize this definition from particles and stiffnesses.
	void Initialize(b3Particle* particle1, b3Particle* particle2, scalar stiffness, scalar dampingStiffness);

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Natural spring length
	scalar length;

	// Spring stiffness
	scalar stiffness;

	// Damping stiffness
	scalar dampingStiffness;
};

// A spring force acting on two particles to maintain them at a desired distance.
class b3SpringForce : public b3Force
{
public:
	// Does this force contain a given particle?
	bool HasParticle(const b3Particle* particle) const;

	// Get the particle 1.
	const b3Particle* GetParticle1() const { return m_p1; }
	b3Particle* GetParticle1() { return m_p1; }

	// Get the particle 2.
	const b3Particle* GetParticle2() const { return m_p2; }
	b3Particle* GetParticle2() { return m_p2; }

	// Set the spring natural rest length.
	void SetLength(scalar length);

	// Get the spring natural rest length.
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
	b3Vec3 GetActionForce() const;

	// Get the force acting on particle 2.
	b3Vec3 GetReactionForce() const;
private:
	friend class b3Force;
	
	b3SpringForce(const b3SpringForceDef* def);
	
	void ClearForces();
	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3Particle* m_p1;

	// Particle 2
	b3Particle* m_p2;

	// Spring natural rest length
	scalar m_L0;

	// Spring stiffness
	scalar m_ks;

	// Damping stiffness
	scalar m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2;
};

inline void b3SpringForce::SetLength(scalar length)
{
	B3_ASSERT(length >= scalar(0));
	m_L0 = length;
}

inline scalar b3SpringForce::GetLength() const
{
	return m_L0;
}

inline void b3SpringForce::SetStiffness(scalar stiffness)
{
	B3_ASSERT(stiffness >= scalar(0));
	m_ks = stiffness;
}

inline scalar b3SpringForce::GetStiffness() const
{
	return m_ks;
}

inline void b3SpringForce::SetDampingStiffness(scalar dampingStiffness)
{
	B3_ASSERT(dampingStiffness >= scalar(0));
	m_kd = dampingStiffness;
}

inline scalar b3SpringForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3SpringForce::GetActionForce() const
{
	return m_f1;
}

inline b3Vec3 b3SpringForce::GetReactionForce() const
{
	return m_f2;
}

#endif
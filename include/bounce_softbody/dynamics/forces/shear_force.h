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

#ifndef B3_SHEAR_FORCE_H
#define B3_SHEAR_FORCE_H

#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/common/math/vec3.h>

// Shear force definition.
// This requires defining the (u, v) coordinates 
// of the triangle and some parameters.
struct b3ShearForceDef : public b3ForceDef
{
	b3ShearForceDef()
	{
		type = e_shearForce;
		u1 = scalar(0);
		v1 = scalar(0);
		u2 = scalar(0);
		v2 = scalar(0);
		u3 = scalar(0);
		v3 = scalar(0);
		stiffness = scalar(0);
		dampingStiffness = scalar(0);
	}
	
	// Initialize the (u, v) coordinates from rest vertices.
	void Initialize(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3);
	
	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Particle 3
	b3Particle* p3;

	// (u, v) coordinates for vertex 1 in the rest state
	scalar u1, v1;
	
	// (u, v) coordinates for vertex 2 in the rest state
	scalar u2, v2;
	
	// (u, v) coordinates for vertex 3 in the rest state
	scalar u3, v3;

	// Shearing stiffness
	scalar stiffness;

	// Damping stiffness
	scalar dampingStiffness;
};

// Shear force acting on a triangle.
class b3ShearForce : public b3Force
{
public:
	// Has this force a given particle?
	bool HasParticle(const b3Particle* particle) const;

	// Get the particle 1.
	const b3Particle* GetParticle1() const { return m_p1; }
	b3Particle* GetParticle1() { return m_p1; }

	// Get the particle 2.
	const b3Particle* GetParticle2() const { return m_p2; }
	b3Particle* GetParticle2() { return m_p2; }
	
	// Get the particle 3.
	const b3Particle* GetParticle3() const { return m_p3; }
	b3Particle* GetParticle3() { return m_p3; }

	// Set the shearing stiffness.
	void SetStiffness(scalar stiffness);

	// Get the shearing stiffness.
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
private:
	friend class b3Force;

	b3ShearForce(const b3ShearForceDef* def);
	
	void ClearForces();
	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3Particle* m_p1;

	// Particle 2
	b3Particle* m_p2;

	// Particle 3
	b3Particle* m_p3;
	
	// Area
	scalar m_alpha;

	// (u, v) matrix
	scalar m_du1, m_dv1;
	scalar m_du2, m_dv2;
	scalar m_inv_det;

	// dwudx, dwvdx
	b3Vec3 m_dwudx, m_dwvdx;

	// Shearing stiffness
	scalar m_ks;

	// Damping stiffness
	scalar m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3;
};

inline void b3ShearForce::SetStiffness(scalar stiffness)
{
	B3_ASSERT(stiffness >= scalar(0));
	m_ks = stiffness;
}

inline scalar b3ShearForce::GetStiffness() const
{
	return m_ks;
}

inline void b3ShearForce::SetDampingStiffness(scalar dampingStiffness)
{
	B3_ASSERT(dampingStiffness >= scalar(0));
	m_kd = dampingStiffness;
}

inline scalar b3ShearForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3ShearForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3ShearForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3ShearForce::GetActionForce3() const
{
	return m_f3;
}

#endif
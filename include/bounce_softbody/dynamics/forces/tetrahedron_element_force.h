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

#ifndef B3_TETRAHEDRON_ELEMENT_FORCE_H
#define B3_TETRAHEDRON_ELEMENT_FORCE_H

#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/common/math/mat33.h>
#include <bounce_softbody/common/math/quat.h>

// Tetrahedral element force definition.
// This requires defining the element in the rest state and its material parameters.
struct b3TetrahedronElementForceDef : public b3ForceDef
{
	b3TetrahedronElementForceDef()
	{
		type = e_tetrahedronElementForce;
		youngModulus = scalar(500);
		poissonRatio = scalar(0.3);
		stiffnessDamping = scalar(0);
	}

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Particle 3
	b3Particle* p3;

	// Particle 4
	b3Particle* p4;
	
	// Vertex 1 in rest state
	b3Vec3 v1;
	
	// Vertex 2 in rest state
	b3Vec3 v2;
	
	// Vertex 3 in rest state
	b3Vec3 v3;
	
	// Vertex 4 in rest state
	b3Vec3 v4;

	// Material Young modulus in [0, inf]
	// Units are 1e3N/m^2
	scalar youngModulus;

	// Material Poisson ratio in [0, 0.5]
	// This is a dimensionless value
	scalar poissonRatio;

	// Coefficient of stiffness damping.
	scalar stiffnessDamping;
};

// Element force acting on a tetrahedron.
class b3TetrahedronElementForce : public b3Force
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

	// Set the material Young Modulus of elasticity in [0, inf].
	void SetYoungModulus(scalar E);
	
	// Get the material Young Modulus of elasticity in [0, inf].
	scalar GetYoungModulus() const;

	// Set the Poisson's ratio in [0, 0.5].
	void SetPoissonRatio(scalar nu);

	// Get the Poisson's ratio in [0, 0.5].
	scalar GetPoissonRatio() const;
	
	// Set the coefficient of stiffness damping.
	void SetStiffnessDamping(scalar damping);

	// Get the coefficient of stiffness damping.
	scalar GetStiffnessDamping() const;
private:
	friend class b3Force;
	
	b3TetrahedronElementForce(const b3TetrahedronElementForceDef* def);

	// This resets the finite element data.
	void ResetElementData();

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

	// Reference tetrahedron
	b3Vec3 m_x1, m_x2, m_x3, m_x4;

	// Reference inverse deformation
	b3Mat33 m_invE;

	// Young Modulus
	scalar m_E;

	// Poisson Ratio
	scalar m_nu;

	// Stiffness matrix in block form
	// This is originally a 12 x 12 matrix
	b3Mat33 m_K[16];
	
	// Rotation of deformation
	b3Quat m_q; 

	// Stiffness damping
	scalar m_stiffnessDamping;
};

inline void b3TetrahedronElementForce::SetYoungModulus(scalar E)
{
	B3_ASSERT(E > scalar(0));
	if (E != m_E)
	{
		m_E = E;
		ResetElementData();
	}
}

inline scalar b3TetrahedronElementForce::GetYoungModulus() const
{
	return m_E;
}

inline void b3TetrahedronElementForce::SetPoissonRatio(scalar nu)
{
	B3_ASSERT(nu >= scalar(0) && nu <= scalar(0.5));
	if (nu != m_nu)
	{
		m_nu = nu;
		ResetElementData();
	}
}

inline scalar b3TetrahedronElementForce::GetPoissonRatio() const
{
	return m_nu;
}

inline void b3TetrahedronElementForce::SetStiffnessDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_stiffnessDamping = damping;
}

inline scalar b3TetrahedronElementForce::GetStiffnessDamping() const
{
	return m_stiffnessDamping;
}

#endif
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

#include <bounce/dynamics/body_solver.h>
#include <bounce/dynamics/force_solver.h>
#include <bounce/dynamics/friction_solver.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/particle.h>
#include <bounce/common/memory/stack_allocator.h>

b3BodySolver::b3BodySolver(const b3BodySolverDef& def)
{
	m_stack = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3Particle**)m_stack->Allocate(m_particleCapacity * sizeof(b3Particle*));

	m_forceCapacity = def.forceCapacity;
	m_forceCount = 0;
	m_forces = (b3Force**)m_stack->Allocate(m_forceCapacity * sizeof(b3Force*));;

	m_shapeContactCapacity = def.shapeContactCapacity;
	m_shapeContactCount = 0;
	m_shapeContacts = (b3SphereAndShapeContact**)m_stack->Allocate(m_shapeContactCapacity * sizeof(b3SphereAndShapeContact*));
}

b3BodySolver::~b3BodySolver()
{
	m_stack->Free(m_shapeContacts);
	m_stack->Free(m_forces);
	m_stack->Free(m_particles);
}

void b3BodySolver::Add(b3Particle* p)
{
	p->m_solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3BodySolver::Add(b3Force* f)
{
	m_forces[m_forceCount++] = f;
}

void b3BodySolver::Add(b3SphereAndShapeContact* c)
{
	m_shapeContacts[m_shapeContactCount++] = c;
}

void b3BodySolver::Solve(const b3TimeStep& step, const b3Vec3& gravity)
{
	{
		// Solve internal dynamics.
		b3ForceSolverDef forceSolverDef;
		forceSolverDef.step = step;
		forceSolverDef.stack = m_stack;
		forceSolverDef.particleCount = m_particleCount;
		forceSolverDef.particles = m_particles;
		forceSolverDef.forceCount = m_forceCount;
		forceSolverDef.forces = m_forces;
		forceSolverDef.shapeContactCount = m_shapeContactCount;
		forceSolverDef.shapeContacts = m_shapeContacts;

		b3ForceSolver forceSolver(forceSolverDef);

		forceSolver.Solve(gravity);
	}

	{
		// Solve friction constraints.
		b3FrictionSolverDef frictionSolverDef;
		frictionSolverDef.step = step;
		frictionSolverDef.shapeContactCount = m_shapeContactCount;
		frictionSolverDef.shapeContacts = m_shapeContacts;
		
		b3FrictionSolver frictionSolver(frictionSolverDef);

		frictionSolver.Solve();
	}
}
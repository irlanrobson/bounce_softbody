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

#include <bounce_softbody/sparse/sparse_force_solver.h>
#include <bounce_softbody/sparse/dense_vec3.h>
#include <bounce_softbody/sparse/diag_mat33.h>
#include <bounce_softbody/sparse/sparse_mat33.h>
#include <bounce_softbody/sparse/sparse_solver.h>

// Time integration using Backward/Implicit Euler:
//
// First order differential equation:
//
// dy/dt = f(t, y(t))
//
// Solution via Backward Euler integration:
//
// y(t + h) = y(t) + h * f(t + h, y(t + h))
//
// Subtract y(t + h) from both sides so we have a non-linear equation in
// standard form:
//
// F(y(t + h)) = y(t) + h * f(t + h, y(t + h)) - y(t + h) = 0
//
// Solution via Newton-Raphson for the i-th iteration:
//
// x_i+1 = x_i - F'(x_i)^-1 * F(x_i)
//
// In practice we solve a linear system for dx if the Jacobian F' is invertible:
//
// dx = x_i+1 - x_i
// F'(x_i) * dx = -F(x_i) 
// x_i+1 = x_i + dx
// 
//
// The ODE we hope solve is:
//
// F = m * a
//
// Transform this second order ODE to two first order ODEs so we can apply a numerical method:
//
// [dx/dt] = [v(t)]
// [dv/dt]   [M^-1 * f(x(t), v(t))]
// 
// Solution via Backward Euler (plus a translation term y):
//
// x(t + h) = x(t) + h * v(t + h) + y
// v(t + h) = v(t) + h * M^-1 * f(x(t + h), v(t + h))
//
// Now we can write a non-linear equation in either x(t + h) or v(t + h).
// 
// We choose to work with velocities for simplifying our constraint satisfaction technique.
// 
// The solution to the non-linear equation in the velocities at the i-th iteration can be equivalently written in this form:
// 
// v_i+1 = v0 + h * M^-1 * f(x_i+1, v_i+1)
// x_i+1 = x0 + h * v_i+1 + y
// 
// Newton-Raphson uses one Taylor expansion around x_i+1 (and v_i+1):
// 
// v_i+1 = v0 + h * M^-1 * (f_i + dfdx_i * dx + dfdv_i * dv)
//
// Introducing dv to the left side:
// 
// v_i + dv = v0 + h * M^-1 * (f_i + dfdx_i * dx + dfdv_i * dv)
// 
// Noting that
// 
// dx = x_i+1 - x_i
// x_i+1 = x0 + h * v_i+1 + y
// x_i+1 = x0 + h * (v_i + dv) + y
//
// we have
//
// dx = [x0 + h * (v_i + dv) + y] - x_i
// 
// We substitute the above in the Newton-Raphson iteration:
//
// v_i + dv = v0 + h * M^-1 * [f_i + dfdx_i * (x0 + h * (v_i + dv) + y - x_i) + dfdv_i * dv]
// 
// Manipulating the equation above, gives a linear equation in dv:
//
// [M - h * dfdv_i - h * h * dfdx_i] * dv = M * (v0 - v_i) + h * f_i + h * dfdx_i * (x0 - x_i + h * v_i + y) 
//
// A = M - h * dfdv_i - h * h * dfdx_i
//
// b = M * (v0 - v_i) + h * f_i + h * dfdx_i * (x0 - x_i + h * v_i + y)
//
void b3SparseSolveBE(b3SolveBEOutput* output, const b3SolveBEInput* input)
{
	scalar h = input->h;
	scalar inv_h = input->inv_h;

	b3SparseForceModel* forceModel = input->forceModel;

	uint32 dofCount = input->dofCount;

	const b3DenseVec3& x0 = *input->x0;
	const b3DenseVec3& v0 = *input->v0;
	const b3DenseVec3& fe = *input->fe;
	const b3DiagMat33& M = *input->M;
	const b3DenseVec3& y = *input->y;
	const b3DiagMat33& S = *input->S;
	const b3DenseVec3& z = *input->z;

	uint32 maxIterations = input->maxIterations;
	scalar epsilon = input->tolerance;

	uint32 maxSubIterations = input->maxSubIterations;
	scalar subEpsilon = input->subTolerance;

	// S^T
	b3DiagMat33 ST(dofCount);
	b3Transpose(ST, S);

	b3DiagMat33 I(dofCount);
	I.SetIdentity();

	// Keep track initial guess.
	b3DenseVec3 py(dofCount);
	py.SetZero();

	b3DenseVec3 x = x0;
	b3DenseVec3 v = v0;

	scalar error0 = scalar(0);
	scalar error = scalar(0);

	uint32 iteration = 0;

	while (iteration < maxIterations)
	{
		b3DenseVec3 fi(dofCount);
		fi.SetZero();

		b3SparseMat33 dfdx(dofCount);
		b3SparseMat33 dfdv(dofCount);

		b3SparseForceSolverData solverData;
		solverData.x = &x;
		solverData.v = &v;
		solverData.f = &fi;
		solverData.dfdx = &dfdx;
		solverData.dfdv = &dfdv;
		solverData.h = h;
		solverData.inv_h = inv_h;

		forceModel->ComputeForces(&solverData);

		b3SparseMat33 A = M - h * dfdv - (h * h) * dfdx;
		
		b3DenseVec3 b = M * (v0 - v) + h * (fe + fi) + h * dfdx * (x0 - x + h * v + y);

		// Pre-filter as in "Smoothed aggregation multigrid for cloth simulation", 
		// by Tamstorf, R., T. Jones, and S. McCormick.
		// A' = S * A * ST + I - S
		// b' = S * (b - A * z)
		b3SparseMat33 pA = S * A * ST + I - S;
		b3DenseVec3 pb = S * (b - A * z);

		// Solve pA * y = pb, 
		// where y = x - z
		b3SolveCGInput subInput;
		subInput.A = &pA;
		subInput.b = &pb;
		subInput.maxIterations = maxSubIterations;
		subInput.tolerance = subEpsilon;

		b3SolveCGOutput subOutput;
		subOutput.x = &py;

		bool subSolved = b3SparseSolveCG(&subOutput, &subInput);
		if (subSolved == false)
		{
			break;
		}

		// Recover x = y + z
		b3DenseVec3 dv = py + z;

		// Track min/max sub-iterations.
		output->minSubIterations = b3Min(output->minSubIterations, subOutput.iterations);
		output->maxSubIterations = b3Max(output->maxSubIterations, subOutput.iterations);

		// Solution update 
		v = v + dv;

		// Position update
		x = x0 + h * v + y;
		
		error = b3LengthSquared(dv);

		if (iteration == 0)
		{
			error0 = error;
			
			if (error <= epsilon * epsilon)
			{
				break;
			}
		}
		else
		{
			if (error <= epsilon * epsilon * error0)
			{
				break;
			}
		}

		++iteration;
	}

	*output->x = x;
	*output->v = v;
	output->iterations = iteration;
	output->error = error;
}

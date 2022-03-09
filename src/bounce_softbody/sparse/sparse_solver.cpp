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

#include <bounce_softbody/sparse/sparse_solver.h>
#include <bounce_softbody/sparse/sparse_mat33.h>
#include <bounce_softbody/sparse/diag_mat33.h>
#include <bounce_softbody/sparse/dense_vec3.h>

// Preconditioned Conjugate Gradient algorithm.
bool b3SparseSolveCG(b3SolveCGOutput* output, const b3SolveCGInput* input)
{
	const b3SparseMat33& A = *input->A;
	const b3DenseVec3& b = *input->b;
	uint32 maxIterations = input->maxIterations;
	scalar epsilon = input->tolerance;
	b3DenseVec3& x = *output->x;

	// Jacobi preconditioner
	// P = diag(A) 
	b3DiagMat33 P(A.rowCount);
	b3DiagMat33 invP(A.rowCount);
	for (uint32 i = 0; i < A.rowCount; ++i)
	{
		b3Mat33 a = A(i, i);

		B3_ASSERT(a.x.x > scalar(0));
		scalar xx = scalar(1) / a.x.x;

		B3_ASSERT(a.y.y > scalar(0));
		scalar yy = scalar(1) / a.y.y;

		B3_ASSERT(a.z.z > scalar(0));
		scalar zz = scalar(1) / a.z.z;

		P[i] = b3Mat33Diagonal(a.x.x, a.y.y, a.z.z);
		invP[i] = b3Mat33Diagonal(xx, yy, zz);
	}

	scalar delta_0 = b3Dot(b, P * b);

	b3DenseVec3 r = b - A * x;
	b3DenseVec3 c = invP * r;

	scalar delta_new = b3Dot(r, c);

	uint32 iteration = 0;
	for (;;)
	{
		if (iteration == maxIterations)
		{
			break;
		}

		if (delta_new <= epsilon * epsilon * delta_0)
		{
			break;
		}

		b3DenseVec3 q = A * c;

		scalar alpha = delta_new / b3Dot(c, q);

		x = x + alpha * c;
		r = r - alpha * q;

		b3DenseVec3 s = invP * r;

		scalar delta_old = delta_new;

		delta_new = b3Dot(r, s);

		scalar beta = delta_new / delta_old;

		c = s + beta * c;

		++iteration;
	}

	output->iterations = iteration;
	output->error = delta_new;
	
	return true;
}
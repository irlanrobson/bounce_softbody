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

#ifndef B3_GEOMETRY_H
#define B3_GEOMETRY_H

#include <bounce_softbody/common/math/vec3.h>

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v) 
// with respect to a segment AB.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(scalar out[3],
	const b3Vec3& A, const b3Vec3& B,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;

	scalar divisor = b3Dot(AB, AB);

	out[0] = b3Dot(QB, AB);
	out[1] = -b3Dot(QA, AB);
	out[2] = divisor;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w) 
// with respect to a triangle ABC.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(scalar out[4],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	scalar divisor = b3Dot(AB_x_AC, AB_x_AC);

	out[0] = b3Dot(QB_x_QC, AB_x_AC);
	out[1] = b3Dot(QC_x_QA, AB_x_AC);
	out[2] = b3Dot(QA_x_QB, AB_x_AC);
	out[3] = divisor;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w, x) 
// with respect to a tetrahedron ABCD.
// The last output value is the (positive) divisor.
inline void b3BarycentricCoordinates(scalar out[5],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;
	b3Vec3 AD = D - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;
	b3Vec3 QD = D - Q;

	scalar divisor = b3Det(AB, AC, AD);
	scalar sign = b3Sign(divisor);

	out[0] = sign * b3Det(QB, QC, QD);
	out[1] = sign * b3Det(QA, QD, QC);
	out[2] = sign * b3Det(QA, QB, QD);
	out[3] = sign * b3Det(QA, QC, QB);
	out[4] = sign * divisor;
}

// Compute the closest point on a segment AB to a point Q.
inline b3Vec3 b3ClosestPointOnSegment(const b3Vec3& A, const b3Vec3& B,
	const b3Vec3& Q)
{
	// Test vertex regions
	scalar wAB[3];
	b3BarycentricCoordinates(wAB, A, B, Q);

	if (wAB[2] == scalar(0))
	{
		return A;
	}

	// R A
	if (wAB[1] <= scalar(0))
	{
		return A;
	}

	// R B
	if (wAB[0] <= scalar(0))
	{
		return B;
	}

	// R AB
	return (wAB[0] * A + wAB[1] * B) / wAB[2];
}

// Compute the closest point on a triangle ABC to a point Q.
inline b3Vec3 b3ClosestPointOnTriangle(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// If the point is on the triangle return projection.
	if (wABC[0] > scalar(0) && wABC[1] > scalar(0) && wABC[2] > scalar(0) && wABC[3] > scalar(0))
	{
		return (wABC[0] * A + wABC[1] * B + wABC[2] * C) / wABC[3];
	}

	b3Vec3 closestPoint = Q;
	scalar closestDistanceSquared = B3_MAX_SCALAR;

	b3Vec3 cAB = b3ClosestPointOnSegment(A, B, Q);
	scalar dAB = b3DistanceSquared(cAB, Q);
	if (dAB < closestDistanceSquared)
	{
		closestDistanceSquared = dAB;
		closestPoint = cAB;
	}

	b3Vec3 cBC = b3ClosestPointOnSegment(B, C, Q);
	scalar dBC = b3DistanceSquared(cBC, Q);
	if (dBC < closestDistanceSquared)
	{
		closestDistanceSquared = dBC;
		closestPoint = cBC;
	}

	b3Vec3 cCA = b3ClosestPointOnSegment(C, A, Q);
	scalar dCA = b3DistanceSquared(cCA, Q);
	if (dCA < closestDistanceSquared)
	{
		closestDistanceSquared = dCA;
		closestPoint = cCA;
	}

	return closestPoint;
}

// Compute the closest point on a tetrahedron ABCD to a point Q.
inline b3Vec3 b3ClosestPointOnTetrahedron(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D,
	const b3Vec3& Q)
{
	scalar wABCD[5];
	b3BarycentricCoordinates(wABCD, A, B, C, D, Q);

	// If the point is inside the tetrahedron return itself.
	if (wABCD[0] > scalar(0) && wABCD[1] > scalar(0) && wABCD[2] > scalar(0) && wABCD[3] > scalar(0) && wABCD[4] > scalar(0))
	{
		return Q;
	}

	b3Vec3 closestPoint = Q;
	scalar closestDistanceSquared = B3_MAX_SCALAR;

	// ABC
	b3Vec3 cABC = b3ClosestPointOnTriangle(Q, A, B, C);
	scalar dABC = b3DistanceSquared(cABC, Q);
	if (dABC < closestDistanceSquared)
	{
		closestDistanceSquared = dABC;
		closestPoint = cABC;
	}

	// ACD
	b3Vec3 cACD = b3ClosestPointOnTriangle(Q, A, C, D);
	scalar dACD = b3DistanceSquared(cACD, Q);
	if (dACD < closestDistanceSquared)
	{
		closestDistanceSquared = dACD;
		closestPoint = cACD;
	}

	// ADB
	b3Vec3 cADB = b3ClosestPointOnTriangle(Q, A, D, B);
	scalar dADB = b3DistanceSquared(cADB, Q);
	if (dADB < closestDistanceSquared)
	{
		closestDistanceSquared = dADB;
		closestPoint = cADB;
	}

	// BDC
	b3Vec3 cBDC = b3ClosestPointOnTriangle(Q, B, D, C);
	scalar dBDC = b3DistanceSquared(cBDC, Q);
	if (dBDC < closestDistanceSquared)
	{
		closestDistanceSquared = dBDC;
		closestPoint = cBDC;
	}

	return closestPoint;
}

#endif
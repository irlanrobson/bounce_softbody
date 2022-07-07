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

#include <bounce_softbody/collision/geometry/geometry.h>

b3Vec3 b3ClosestPointOnSegment(const b3Vec3& A, const b3Vec3& B,
	const b3Vec3& Q)
{
	scalar wAB[3];
	b3BarycentricCoordinates(wAB, A, B, Q);

	// If the point is on the segment return projection.
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wAB[2] > scalar(0))
	{
		return (wAB[0] * A + wAB[1] * B) / wAB[2];
	}

	scalar dA = b3DistanceSquared(A, Q);
	scalar dB = b3DistanceSquared(B, Q);

	if (dA < dB)
	{
		return A;
	}
	else
	{
		return B;
	}
}

b3Vec3 b3ClosestPointOnTriangle(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
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

b3Vec3 b3ClosestPointOnTetrahedron(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D,
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

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
	// Test vertex regions
	scalar wAB[3];
	b3BarycentricCoordinates(wAB, A, B, Q);

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
	B3_ASSERT(wAB[2] > scalar(0));
	return (wAB[0] * A + wAB[1] * B) / wAB[2];
}

b3Vec3 b3ClosestPointOnTriangle(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		return A;
	}

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		return B;
	}

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		return C;
	}

	// Test edge regions		
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wABC[3] * wABC[2] <= scalar(0))
	{
		B3_ASSERT(wAB[2] > scalar(0));
		return (wAB[0] * A + wAB[1] * B) / wAB[2];
	}

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && wABC[3] * wABC[0] <= scalar(0))
	{
		B3_ASSERT(wBC[2] > scalar(0));
		return (wBC[0] * B + wBC[1] * C) / wBC[2];
	}

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && wABC[3] * wABC[1] <= scalar(0))
	{
		B3_ASSERT(wCA[2] > scalar(0));
		return (wCA[0] * C + wCA[1] * A) / wCA[2];
	}

	// R ABC/ACB
	B3_ASSERT(wABC[3] > scalar(0));
	return (wABC[0] * A + wABC[1] * B + wABC[2] * C) / wABC[3];
}
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

#include <bounce_softbody/common/math/mat22.h>
#include <bounce_softbody/common/math/mat33.h>
#include <bounce_softbody/common/math/mat44.h>
#include <bounce_softbody/common/math/transform.h>

const b3Vec2 b3Vec2_zero(scalar(0), scalar(0));
const b3Vec2 b3Vec2_x(scalar(1), scalar(0));
const b3Vec2 b3Vec2_y(scalar(0), scalar(1));

const b3Vec3 b3Vec3_zero(scalar(0), scalar(0), scalar(0));
const b3Vec3 b3Vec3_x(scalar(1), scalar(0), scalar(0));
const b3Vec3 b3Vec3_y(scalar(0), scalar(1), scalar(0));
const b3Vec3 b3Vec3_z(scalar(0), scalar(0), scalar(1));

const b3Vec4 b3Vec4_zero(scalar(0), scalar(0), scalar(0), scalar(0));

const b3Mat22 b3Mat22_zero(
	b3Vec2(scalar(0), scalar(0)),
	b3Vec2(scalar(0), scalar(0)));

const b3Mat22 b3Mat22_identity(
	b3Vec2(scalar(1), scalar(0)),
	b3Vec2(scalar(0), scalar(1)));

const b3Mat33 b3Mat33_zero(
	b3Vec3(scalar(0), scalar(0), scalar(0)),
	b3Vec3(scalar(0), scalar(0), scalar(0)),
	b3Vec3(scalar(0), scalar(0), scalar(0)));

const b3Mat33 b3Mat33_identity(
	b3Vec3(scalar(1), scalar(0), scalar(0)),
	b3Vec3(scalar(0), scalar(1), scalar(0)),
	b3Vec3(scalar(0), scalar(0), scalar(1)));

const b3Mat44 b3Mat44_zero(
	b3Vec4(scalar(0), scalar(0), scalar(0), scalar(0)),
	b3Vec4(scalar(0), scalar(0), scalar(0), scalar(0)),
	b3Vec4(scalar(0), scalar(0), scalar(0), scalar(0)),
	b3Vec4(scalar(0), scalar(0), scalar(0), scalar(0)));

const b3Mat44 b3Mat44_identity(
	b3Vec4(scalar(1), scalar(0), scalar(0), scalar(0)),
	b3Vec4(scalar(0), scalar(1), scalar(0), scalar(0)),
	b3Vec4(scalar(0), scalar(0), scalar(1), scalar(0)),
	b3Vec4(scalar(0), scalar(0), scalar(0), scalar(1)));

const b3Transform b3Transform_identity(b3Vec3_zero, b3Quat_identity);

const b3Quat b3Quat_identity(b3Vec3_zero, scalar(1));

b3Vec2 b3Mat22::Solve(const b3Vec2& b) const
{
	// Cramer's rule
	scalar a11 = x.x, a12 = y.x;
	scalar a21 = x.y, a22 = y.y;

	scalar det = a11 * a22 - a12 * a21;

	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}

	b3Vec2 xn;
	xn.x = det * (a22 * b.x - a12 * b.y);
	xn.y = det * (a11 * b.y - a21 * b.x);
	return xn;
}

b3Mat22 b3Inverse(const b3Mat22& A)
{
	scalar det = b3Det(A.x, A.y);
	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}
	return det * b3Adjugate(A);
}

b3Vec3 b3Mat33::Solve(const b3Vec3& b) const
{
	// Cramer's rule
	scalar det = b3Det(x, y, z);
	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}
	
	b3Vec3 xn;
	xn.x = det * b3Det(b, y, z);
	xn.y = det * b3Det(x, b, z);
	xn.z = det * b3Det(x, y, b);
	return xn;
}

b3Mat33 b3Inverse(const b3Mat33& A)
{
	// Cofactor method
	scalar det = b3Det(A.x, A.y, A.z);
	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}
	return det * b3Adjugate(A);
}

b3Mat33 b3SymInverse(const b3Mat33& A)
{
	scalar det = b3Det(A.x, A.y, A.z);
	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}

	scalar a11 = A.x.x, a12 = A.y.x, a13 = A.z.x;
	scalar a22 = A.y.y, a23 = A.z.y;
	scalar a33 = A.z.z;

	b3Mat33 M;

	M.x.x = det * (a22 * a33 - a23 * a23);
	M.x.y = det * (a13 * a23 - a12 * a33);
	M.x.z = det * (a12 * a23 - a13 * a22);

	M.y.x = M.x.y;
	M.y.y = det * (a11 * a33 - a13 * a13);
	M.y.z = det * (a13 * a12 - a11 * a23);

	M.z.x = M.x.z;
	M.z.y = M.y.z;
	M.z.z = det * (a11 * a22 - a12 * a12);

	return M;
}

static B3_FORCE_INLINE scalar b3Minor(const b3Mat44& A, uint32 i, uint32 j)
{
	b3Mat33 S;
	b3SubMatrix(&S.x.x, &A.x.x, 4, 4, i, j);
	return b3Det(S.x, S.y, S.z);
}

// (-1)^(i + j)
static B3_FORCE_INLINE int32 b3CofactorSign(uint32 i, uint32 j)
{
	return (i + j) % 2 == 0 ? 1 : -1;
}

static B3_FORCE_INLINE scalar b3Cofactor(const b3Mat44& A, uint32 i, uint32 j)
{
	int32 sign = b3CofactorSign(i, j);
	scalar minor = b3Minor(A, i, j);
	return scalar(sign) * minor;
}

static B3_FORCE_INLINE b3Mat44 b3CofactorMatrix(const b3Mat44& A)
{
	b3Mat44 C;
	for (uint32 i = 0; i < 4; ++i)
	{
		for (uint32 j = 0; j < 4; ++j)
		{
			C(i, j) = b3Cofactor(A, i, j);
		}
	}
	return C;
}

static B3_FORCE_INLINE scalar b3Det(const b3Mat44& A, const b3Mat44& C)
{
	// Cofactor expansion along the first row
	scalar result = scalar(0);
	for (uint32 j = 0; j < 4; ++j)
	{
		result += A(0, j) * C(0, j);
	}
	return result;
}

b3Mat44 b3Inverse(const b3Mat44& A)
{
	// Cofactor matrix
	b3Mat44 C = b3CofactorMatrix(A);

	scalar det = b3Det(A, C);

	if (det != scalar(0))
	{
		det = scalar(1) / det;
	}
	
	// A^-1 = |A|^-1 * adj(A) = |A|^-1 * C^T
	return det * b3Transpose(C);
}
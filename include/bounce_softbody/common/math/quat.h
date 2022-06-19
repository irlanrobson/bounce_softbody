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

#ifndef B3_QUAT_H
#define B3_QUAT_H

#include <bounce_softbody/common/math/math.h>
#include <bounce_softbody/common/math/mat33.h>

// A quaternion can represent an orientation with 4 scalars.
struct b3Quat
{
	// Default constructor does nothing for performance.
	b3Quat() { }

	// Construct this quaternion from a vector part and a scalar part.
	b3Quat(const b3Vec3& _v, scalar _s) : v(_v), s(_s) { }

	// Add a quaternion to this quaternion.
	void operator+=(const b3Quat& q)
	{
		v += q.v;
		s += q.s;
	}

	// Subtract a quaternion from this quaternion.
	void operator-=(const b3Quat& q)
	{
		v -= q.v;
		s -= q.s;
	}

	// Set this quaternion to identity.
	void SetIdentity()
	{
		v.SetZero();
		s = scalar(1);
	}

	// Set this quaternion from a vector part and a scalar part.
	void Set(const b3Vec3& _v, scalar _s)
	{
		v = _v;
		s = _s;
	}

	// Convert this quaternion to the unit quaternion. Return the length.
	scalar Normalize()
	{
		scalar len = b3Sqrt(b3Dot(v, v) + s * s);
		if (len > B3_EPSILON)
		{
			v /= len;
			s /= len;
		}
		return len;
	}

	// Rotate a vector.
	b3Vec3 Rotate(const b3Vec3& vIn) const
	{
		b3Vec3 t = scalar(2) * b3Cross(v, vIn);
		return vIn + s * t + b3Cross(v, t);
	}

	// Set this quaternion from an axis and full angle 
	// of rotation about the axis.
	void SetAxisAngle(const b3Vec3& axis, scalar angle)
	{
		scalar theta = scalar(0.5) * angle;
		v = sin(theta) * axis;
		s = cos(theta);
	}

	// If this quaternion represents an orientation output 
	// the axis and angle of rotation about the axis.
	void GetAxisAngle(b3Vec3* axis, scalar* angle) const
	{
		// sin^2 = 1 - cos^2
		// sin = sqrt( sin^2 ) = ||v||
		// axis = v / sin
		scalar sine = b3Length(v);
		axis->SetZero();
		if (sine > B3_EPSILON)
		{
			*axis = v / sine;
		}

		// Ensure cosine is in the range [-1, 1]
		scalar cosine = b3Clamp(s, scalar(-1), scalar(1));

		// Half angle
		scalar theta = acos(cosine);

		// Full angle
		*angle = scalar(2) * theta;
	}

	// Get the x axis.
	b3Vec3 GetXAxis() const
	{
		return Rotate(b3Vec3_x);
	}

	// Get the y axis.
	b3Vec3 GetYAxis() const
	{
		return Rotate(b3Vec3_y);
	}

	// Get the z axis.
	b3Vec3 GetZAxis() const
	{
		return Rotate(b3Vec3_z);
	}

	// Get the x, y, z axes.
	b3Mat33 GetRotationMatrix() const
	{
		scalar x = v.x, y = v.y, z = v.z, w = s;

		scalar x2 = x + x, y2 = y + y, z2 = z + z;
		scalar xx = x * x2, xy = x * y2, xz = x * z2;
		scalar yy = y * y2, yz = y * z2, zz = z * z2;
		scalar wx = w * x2, wy = w * y2, wz = w * z2;

		return b3Mat33(
			b3Vec3(scalar(1) - (yy + zz), xy + wz, xz - wy),
			b3Vec3(xy - wz, scalar(1) - (xx + zz), yz + wx),
			b3Vec3(xz + wy, yz - wx, scalar(1) - (xx + yy)));
	}

	// Get the angle about the x axis.
	scalar GetXAngle() const
	{
		return scalar(2) * atan2(v.x, s);
	}

	// Get the angle about the y axis.
	scalar GetYAngle() const
	{
		return scalar(2) * atan2(v.y, s);
	}

	// Get the angle about the z axis.
	scalar GetZAngle() const
	{
		return scalar(2) * atan2(v.z, s);
	}

	b3Vec3 v;
	scalar s;
};

// Identity quaternion
extern const b3Quat b3Quat_identity;

// Add two quaternions.
inline b3Quat operator+(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(a.v + b.v, a.s + b.s);
}

// Subtract two quaternions.
inline b3Quat operator-(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(a.v - b.v, a.s - b.s);
}

// Multiply a quaternion by a scalar.
inline b3Quat operator*(scalar s, const b3Quat& q)
{
	return b3Quat(s * q.v, s * q.s);
}

// Divide a quaternion by a scalar.
inline b3Quat operator/(const b3Quat& q, scalar s)
{
	return b3Quat(q.v / s, q.s / s);
}

// Negate a quaternion.
inline b3Quat operator-(const b3Quat& q)
{
	return b3Quat(-q.v, -q.s);
}

// Multiply two quaternions.
inline b3Quat b3Mul(const b3Quat& a, const b3Quat& b)
{
	b3Quat result;
	result.v = b3Cross(a.v, b.v) + a.s * b.v + b.s * a.v;
	result.s = a.s * b.s - b3Dot(a.v, b.v);
	return result;
}

// Multiply two quaternions.
inline b3Quat operator*(const b3Quat& a, const b3Quat& b)
{
	return b3Mul(a, b);
}

// Perform the dot poduct of two quaternions.
inline scalar b3Dot(const b3Quat& a, const b3Quat& b)
{
	return b3Dot(a.v, b.v) + a.s * b.s;
}

// Return the conjugate of a quaternion.
// If the quaternion is a rotation this returns its inverse.
inline b3Quat b3Conjugate(const b3Quat& q)
{
	return b3Quat(-q.v, q.s);
}

// Multiply the conjugate of a quaternion times another quaternion.
inline b3Quat b3MulC(const b3Quat& a, const b3Quat& b)
{
	return b3Mul(b3Conjugate(a), b);
}

// Return the length of a quaternion.
inline scalar b3Length(const b3Quat& q)
{
	return b3Sqrt(b3Dot(q.v, q.v) + q.s * q.s);
}

// Convert a quaternion to the unit quaternion.
inline b3Quat b3Normalize(const b3Quat& q)
{
	scalar len = b3Length(q);
	if (len > B3_EPSILON)
	{
		return q / len;
	}
	return b3Quat_identity;
}

// Rotate a vector.
inline b3Vec3 b3Mul(const b3Quat& q, const b3Vec3& v)
{
	return q.Rotate(v);
}

// Inverse rotate a vector.
inline b3Vec3 b3MulC(const b3Quat& q, const b3Vec3& v)
{
	return b3Mul(b3Conjugate(q), v);
}

// Convert a 3-by3 rotation matrix to a rotation quaternion.
inline b3Quat b3Mat33Quat(const b3Mat33& m)
{
	// Check the diagonal.
	scalar trace = m[0][0] + m[1][1] + m[2][2];

	if (trace > scalar(0))
	{
		b3Quat result;

		scalar s = b3Sqrt(trace + scalar(1));
		result.s = scalar(0.5) * s;

		scalar t = scalar(0.5) / s;
		result.v.x = t * (m[1][2] - m[2][1]);
		result.v.y = t * (m[2][0] - m[0][2]);
		result.v.z = t * (m[0][1] - m[1][0]);

		return result;
	}

	// Diagonal is negative.
	const uint32 next[3] = { 1, 2, 0 };

	uint32 i = 0;

	if (m[1][1] > m[0][0])
	{
		i = 1;
	}

	if (m[2][2] > m[i][i])
	{
		i = 2;
	}

	uint32 j = next[i];
	uint32 k = next[j];

	scalar s = b3Sqrt((m[i][i] - (m[j][j] + m[k][k])) + scalar(1));

	scalar q[4];
	q[i] = s * scalar(0.5);

	scalar t;
	if (s != scalar(0))
	{
		t = scalar(0.5) / s;
	}
	else
	{
		t = s;
	}

	q[3] = t * (m[j][k] - m[k][j]);
	q[j] = t * (m[i][j] + m[j][i]);
	q[k] = t * (m[i][k] + m[k][i]);

	b3Quat result;
	result.v.x = q[0];
	result.v.y = q[1];
	result.v.z = q[2];
	result.s = q[3];
	return result;
}

// Rotation about the x-axis.
inline b3Quat b3QuatRotationX(scalar angle)
{
	scalar x = scalar(0.5) * angle;

	b3Quat q;
	q.v.x = sin(x);
	q.v.y = scalar(0);
	q.v.z = scalar(0);
	q.s = cos(x);
	return q;
}

// Rotation about the y-axis.
inline b3Quat b3QuatRotationY(scalar angle)
{
	scalar x = scalar(0.5) * angle;

	b3Quat q;
	q.v.x = scalar(0);
	q.v.y = sin(x);
	q.v.z = scalar(0);
	q.s = cos(x);
	return q;
}

// Rotation about the z-axis.
inline b3Quat b3QuatRotationZ(scalar angle)
{
	scalar x = scalar(0.5) * angle;

	b3Quat q;
	q.v.x = scalar(0);
	q.v.y = scalar(0);
	q.v.z = sin(x);
	q.s = cos(x);
	return q;
}

// Rotation between two normal vectors.
inline b3Quat b3QuatRotationBetween(const b3Vec3& a, const b3Vec3& b)
{
	b3Quat q;

	// a = -b 
	// a + b = 0
	if (b3LengthSquared(a + b) <= B3_EPSILON * B3_EPSILON)
	{
		// Opposite vectors
		// Choose 180 degree rotation about a perpendicular axis
		// sin(pi / 2) = 1
		// cos(pi / 2) = 0
		q.v = b3Perp(a);
		q.s = scalar(0);
	}
	else
	{
		// |a x b| = sin(theta)
		// a . b = cos(theta)
		// sin(theta / 2) = +/- sqrt([1 - cos(theta)] / 2)
		// cos(theta / 2) = +/- sqrt([1 + cos(theta)] / 2)
		// q.v = sin(theta / 2) * (a x b) / |a x b|
		// q.s = cos(theta / 2)
		b3Vec3 axis = b3Cross(a, b);
		scalar s = b3Length(axis);
		scalar c = b3Dot(a, b);
		if (s <= B3_EPSILON)
		{
			// Paralell vectors
			q.SetIdentity();
		}
		else
		{
			q.v = b3Sqrt(scalar(0.5) * (scalar(1) - c)) * (axis / s);
			q.s = b3Sqrt(scalar(0.5) * (scalar(1) + c));
		}
	}

	return q;
}

#endif
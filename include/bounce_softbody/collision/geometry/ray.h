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

#ifndef B3_RAY_H
#define B3_RAY_H

#include <bounce_softbody/common/math/vec3.h>

// A ray in finite form.
// R(t) = O + t * n.
struct b3Ray
{
	// Default ctor does nothing for performance.
	b3Ray() { }
	
	// Construct this ray from origin, direction, and length.
	b3Ray(const b3Vec3& _origin, const b3Vec3& _direction, scalar _length) : 
		origin(_origin), direction(_direction), length(_length) { }

	// Construct this ray from a line segment.
	b3Ray(const b3Vec3& A, const b3Vec3& B) 
	{
		origin = A;
		direction = B - A;
		length = direction.Normalize();
	}

	// Return the begin point of this ray.
	b3Vec3 A() const { return origin; }

	// Return the end point of this ray.
	b3Vec3 B() const
	{
		return origin + length * direction;
	}

	b3Vec3 origin;
	b3Vec3 direction;
	scalar length;
};

#endif
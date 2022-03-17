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

#ifndef B3_SPHERE_H
#define B3_SPHERE_H

#include <bounce_softbody/common/math/vec3.h>

struct b3Sphere
{
	b3Vec3 vertex;
	scalar radius;

	b3Sphere() { }
	
	b3Sphere(const b3Vec3& v, scalar r)
	{
		vertex = v;
		radius = r;
	}

	~b3Sphere() { }

	const b3Vec3& GetVertex(uint32 index) const;
	uint32 GetSupportVertex(const b3Vec3& direction) const;
};

inline const b3Vec3& b3Sphere::GetVertex(uint32 index) const
{
    B3_NOT_USED(index);
	return vertex;
}

inline uint32 b3Sphere::GetSupportVertex(const b3Vec3& direction) const
{
	B3_NOT_USED(direction);
	return 0;
}

#endif
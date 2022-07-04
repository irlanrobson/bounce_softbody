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

#ifndef B3_INDEX_3D_H
#define B3_INDEX_3D_H

#include <bounce_softbody/common/settings.h>

// The index of an element in a generic three-dimensional matrix structure.
struct b3Index3D
{
	// The actual integer type of the stored indices.
	using ValueType = int64;

	// Default constructor does nothing for performance.
	b3Index3D() { }

	// Construct this 3D index from three values.
	b3Index3D(ValueType _i, ValueType _j, ValueType _k)
	{
		i = _i;
		j = _j;
		k = _k;
	}

	// Converts this three-dimensional index into an unidimensional value.
	// This is usefull if you store your matrix data in a single unidimensional array.
	ValueType GetOneDimensionalIndex(ValueType iSize, ValueType jSize) const
	{
		return i +
			j * iSize +
			k * iSize * jSize;
	}

	// The three indices.
	ValueType i, j, k;
};

#endif

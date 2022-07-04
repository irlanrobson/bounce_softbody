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

#ifndef B3_REGULAR_GRID_INDEXER_H
#define B3_REGULAR_GRID_INDEXER_H

#include <bounce_softbody/collision/geometry/aabb.h>
#include <bounce_softbody/collision/geometry/index_3d.h>

// Cells indexer.
// This is a helper structure to handle the cells in a regular grid.
// This grid indexer does not contain any grid data. It was designed  
// to be used in an actual regular grid implementation.
struct b3RegularGridIndexer 
{
	// Default ctor creates an invalid indexer.
	b3RegularGridIndexer() 
	{
		aabb.lowerBound.SetZero();
		aabb.upperBound.SetZero();
		width = 0;
		height = 0;
		depth = 0;
	}
	
	// Construct this grid from AABB and dimensions.
	b3RegularGridIndexer(const b3AABB& _aabb, uint32 _width, uint32 _height, uint32 _depth)
	{
		aabb = _aabb;
		width = _width;
		height = _height;
		depth = _depth;
	}

	// Does the given index points to a cell that is inside this grid?
	bool Contains(const b3Index3D& index) const
	{
		return index.i >= 0 && index.i < width &&
			index.j >= 0 && index.j < height &&
			index.k >= 0 && index.k < depth;
	}

	// Get the origin of this grid.
	b3Vec3 GetOrigin() const
	{
		return aabb.lowerBound;
	}

	// Get the grid dimensions.
	b3Vec3 GetDimensions() const
	{
		return b3Vec3(scalar(width), scalar(height), scalar(depth));
	}

	// Get the number of cells in this grid.
	uint32 GetCellCount() const
	{
		return width * height * depth;
	}

	// Get the size of each cell in this grid.
	b3Vec3 GetCellSize() const
	{
		return aabb.GetDimensions() / GetDimensions();
	}

	// Get the radius vector (half-extents) of each cell in this grid.
	b3Vec3 GetCellRadius() const
	{
		return GetCellSize() / scalar(2);
	}

	// Get the center of the cell at the given index.
	b3Vec3 GetCellCenter(const b3Index3D& index) const
	{
		b3Vec3 cellSize = GetCellSize();
		b3Vec3 localPoint = b3Vec3(index.i * cellSize.x, index.j * cellSize.y, index.k * cellSize.z);
		return GetOrigin() + localPoint + cellSize / scalar(2);
	}
	
	// Get the AABB of the cell at the given index.
	b3AABB GetCellAABB(const b3Index3D& index) const
	{
		return b3AABB(GetCellCenter(index), GetCellRadius());
	}

	// Get the index of the cell where the given point is in.
	b3Index3D GetCellIndex(const b3Vec3& point) const
	{
		b3Vec3 cellSize = GetCellSize();
		b3Vec3 localPoint = point - GetOrigin();
		b3Vec3 cellPoint = localPoint / cellSize;
		return b3Index3D(floor(cellPoint.x), floor(cellPoint.y), floor(cellPoint.z));
	}

	// Converts a three-dimensional cell index to an unidimensional value.
	// This is usefull if you store all your grid data in a single unidimensional array.
	uint32 GetOneDimensionalIndex(const b3Index3D& index) const
	{
		B3_ASSERT(Contains(index));
		return uint32(index.GetOneDimensionalIndex(width, height));
	}

	b3AABB aabb;
	uint32 width;
	uint32 height;
	uint32 depth;
};

#endif

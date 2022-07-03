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

/*
* MIT License
*
* Copyright (c) 2019 "O Programador"

* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef B3_VOXEL_GRID_H
#define B3_VOXEL_GRID_H

#include <bounce_softbody/collision/geometry/regular_grid_indexer.h>

// A regular grid of "voxels" aka "3D pixels". 
// See https://en.wikipedia.org/wiki/Voxel.
// Based on https://github.com/oprogramadorreal/vize's VoxelGrid. 
// O Programador's license is copied over above. 
template <typename T>
class b3VoxelGrid 
{
public:
	// Constructs an empty voxel grid.
	b3VoxelGrid()
	{
		m_voxels = nullptr;
		m_voxelCount = 0;
	}

	// The destructor frees the voxel data using b3Free.
	~b3VoxelGrid()
	{
		Clear();
	}

	// Clear all data.
	// This function uses b3Free to free the voxel data.
	void Clear()
	{
		b3Free(m_voxels);
		m_voxels = nullptr;
		m_voxelCount = 0;
	}

	// Creates a grid bounded by the given AABB and grid dimensions.
	// This function uses b3Alloc to allocate the voxel data.
	void Create(const b3AABB& aabb, uint32 width, uint32 height, uint32 depth)
	{
		B3_ASSERT(m_voxels == nullptr && m_voxelCount == 0);
		
		B3_ASSERT(width > 1);
		B3_ASSERT(height > 1);
		B3_ASSERT(depth > 1);

		m_indexer = b3RegularGridIndexer(aabb, width - 1, height - 1, depth - 1);
		m_voxelCount = width * height * depth;
		m_voxels = (T*)b3Alloc(m_voxelCount * sizeof(T));
	}

	// Return the width of this grid in number of voxels.
	uint32 GetWidth() const
	{
		return m_indexer.width + 1;
	}

	// Return the height of this grid in number of voxels.
	uint32 GetHeight() const
	{
		return m_indexer.height + 1;
	}

	// Return the depth of this grid in number of voxels.
	uint32 GetDepth() const
	{
		return m_indexer.depth + 1;
	}

	// Return the voxel data.
	const T* GetVoxelData() const
	{
		return m_voxels;
	}

	// Get the number of voxels.
	uint32 GetVoxelCount() const
	{
		return m_voxelCount;
	}

	// Write the voxel at a given index.
	void SetVoxel(uint32 index, const T& value)
	{
		B3_ASSERT(index < m_voxelCount);
		m_voxels[index] = value;
	}

	// Read the voxel at a given index.
	const T& GetVoxel(uint32 index) const
	{
		B3_ASSERT(index < m_voxelCount);
		return m_voxels[index];
	}

	// Write the voxel at a given 3D index.
	void SetVoxel(const b3Index3D& index, const T& value)
	{
		B3_ASSERT(ContainsVoxel(index));
		uint32 voxelIndex = GetVoxelIndex(index);
		B3_ASSERT(voxelIndex < m_voxelCount);
		m_voxels[voxelIndex] = value;
	}

	// Read the voxel at a given 3D index.
	const T& GetVoxel(const b3Index3D& index) const
	{
		B3_ASSERT(ContainsVoxel(index));
		uint32 voxelIndex = GetVoxelIndex(index);
		B3_ASSERT(voxelIndex < m_voxelCount);
		return m_voxels[voxelIndex];
	}

	// Return the position in for the voxel at the given voxel index.
	b3Vec3 GetVoxelPosition(const b3Index3D& index) const
	{
		B3_ASSERT(ContainsVoxel(index));
		return m_indexer.GetCellAABB(index).lowerBound;
	}

	// Return an interpolated voxel value at the given point inside this grid.
	// The point must be inside this grid. Call Contains() to verify if the point is inside the grid.
	T Sample(const b3Vec3& point) const
	{
		b3Index3D cellIndex = m_indexer.GetCellIndex(point);
		B3_ASSERT(m_indexer.Contains(cellIndex));

		b3AABB cellAABB = GetCellAABB(cellIndex);
		b3Vec3 relPoint = cellAABB.GetRelativePosition(point);

		T cellVoxels[8];
		GetCellVoxels(cellVoxels, cellIndex);

		// Lerp 
		return InterpolateVoxel(relPoint, cellVoxels);
	}

	// Return an interpolated gradient at the given point inside this grid.
	// The point must be inside this grid. Call Contains() to verify if the point is inside the grid.
	// Note: If the gradient is a surface normal don't forget to normalize the value!
	b3Vec3 SampleGradient(const b3Vec3& point) const
	{
		b3Index3D cellIndex = m_indexer.GetCellIndex(point);
		B3_ASSERT(m_indexer.Contains(cellIndex));

		b3AABB cellAABB = GetCellAABB(cellIndex);
		b3Vec3 relPoint = cellAABB.GetRelativePosition(point);

		T cellVoxels[8];
		GetCellVoxels(cellVoxels, cellIndex);

		b3Vec3 gradient;
		gradient.x = InterpolateVoxel(b3Vec3(1, relPoint.y, relPoint.z), cellVoxels) - InterpolateVoxel(b3Vec3(0, relPoint.y, relPoint.z), cellVoxels);
		gradient.y = InterpolateVoxel(b3Vec3(relPoint.x, 1, relPoint.z), cellVoxels) - InterpolateVoxel(b3Vec3(relPoint.x, 0, relPoint.z), cellVoxels);
		gradient.z = InterpolateVoxel(b3Vec3(relPoint.x, relPoint.y, 1), cellVoxels) - InterpolateVoxel(b3Vec3(relPoint.x, relPoint.y, 0), cellVoxels);
		return gradient;
	}

	// Get the width of this grid in number of cells.
	uint32 GetWidthInCells() const
	{
		return m_indexer.width;
	}
	
	// Get the height of this grid in number of cells.
	uint32 GetHeightInCells() const
	{
		return m_indexer.height;
	}
	
	// Get the depth of this grid in number of cells.
	uint32 GetDepthInCells() const
	{
		return m_indexer.depth;
	}
	
	// Get one full cell (8 voxels) of this grid.
	void GetCellVoxels(T voxels[8], const b3Index3D& cellIndex) const
	{
		B3_ASSERT(ContainsCell(cellIndex));

		b3Index3D::IndexType i = cellIndex.i;
		b3Index3D::IndexType j = cellIndex.j;
		b3Index3D::IndexType k = cellIndex.k;

		voxels[0] = GetVoxel(b3Index3D(i, j, k));
		voxels[1] = GetVoxel(b3Index3D(i, j, k + 1));
		voxels[2] = GetVoxel(b3Index3D(i, j + 1, k));
		voxels[3] = GetVoxel(b3Index3D(i, j + 1, k + 1));
		voxels[4] = GetVoxel(b3Index3D(i + 1, j, k));
		voxels[5] = GetVoxel(b3Index3D(i + 1, j, k + 1));
		voxels[6] = GetVoxel(b3Index3D(i + 1, j + 1, k));
		voxels[7] = GetVoxel(b3Index3D(i + 1, j + 1, k + 1));
	}

	// Return he AABB of the cell specified by cellIndex.
	b3AABB GetCellAABB(const b3Index3D& cellIndex) const
	{
		return m_indexer.GetCellAABB(cellIndex);
	}

	// Get the index of the cell where point lies in.
	b3Index3D GetCellIndexOfPoint(const b3Vec3& point) const
	{
		return m_indexer.GetCellIndex(point);
	}

	// Get the bounding box for this grid.
	const b3AABB& GetAABB() const
	{
		return m_indexer.aabb;
	}

	// Does the given cell index point to a cell that is logically inside this grid?
	bool ContainsCell(const b3Index3D& cellIndex) const
	{
		return m_indexer.Contains(cellIndex);
	}

	// Does the given cell index point to a cell that is logically inside this grid?
	bool ContainsVoxel(const b3Index3D& voxelIndex) const
	{
		// Note: This converts an unsigned 32 bit integer to a signed 64 bit integer.
		return
			voxelIndex.i >= b3Index3D::IndexType(0) && voxelIndex.i < b3Index3D::IndexType(GetWidth()) &&
			voxelIndex.j >= b3Index3D::IndexType(0) && voxelIndex.j < b3Index3D::IndexType(GetHeight()) &&
			voxelIndex.k >= b3Index3D::IndexType(0) && voxelIndex.k < b3Index3D::IndexType(GetDepth());
	}

	 // Is the given point inside the AABB of this grid?
	bool Contains(const b3Vec3& point) const
	{
		return ContainsCell(GetCellIndexOfPoint(point));
	}
private:
	// Convert a given 3D index to a 1D voxel index.
	uint32 GetVoxelIndex(const b3Index3D& index) const
	{
		// Note: This converts an signed 64 bit integer to an unsigned 32 bit integer.
		uint32 index1D = index.GetOneDimensionalIndex(GetWidth(), GetHeight(), GetDepth());
		B3_ASSERT(index1D >= 0);
		return index1D;
	}

	// Trilinear interpolation given relative point inside the cell AABB and 8 voxels around the point.
	// Based on http://en.wikipedia.org/wiki/Trilinear_interpolation
	T InterpolateVoxel(const b3Vec3& relativePointInCell, const T voxels[8]) const
	{
		// Ensure the point is inside the cell AABB.
		scalar x = b3Clamp(relativePointInCell.x, scalar(0), scalar(1));
		scalar y = b3Clamp(relativePointInCell.y, scalar(0), scalar(1));
		scalar z = b3Clamp(relativePointInCell.z, scalar(0), scalar(1));

		// x interpolation
		scalar c00 = b3LinearInterpolation(x, voxels[0], voxels[4]);
		scalar c10 = b3LinearInterpolation(x, voxels[2], voxels[6]);
		scalar c01 = b3LinearInterpolation(x, voxels[1], voxels[5]);
		scalar c11 = b3LinearInterpolation(x, voxels[3], voxels[7]);

		// y interpolation
		scalar c0 = b3LinearInterpolation(y, c00, c10);
		scalar c1 = b3LinearInterpolation(y, c01, c11);

		// z interpolation
		return b3LinearInterpolation(z, c0, c1);
	}

	// Cells indexer.
	b3RegularGridIndexer m_indexer;

	// The voxel data. Owned by this class.
	T* m_voxels;

	// The voxel count.
	uint32 m_voxelCount;
};

using b3ScalarVoxelGrid = b3VoxelGrid<scalar>;

#endif

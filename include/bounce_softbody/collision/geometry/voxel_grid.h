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
// VIZE's license is copied over above. 
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

		m_cellsIndexer = b3RegularGridIndexer(aabb, width - 1, height - 1, depth - 1);
		m_voxelCount = width * height * depth;
		m_voxels = (T*)b3Alloc(m_voxelCount * sizeof(T));
	}

	// Get the width of this grid in number of voxels.
	uint32 GetWidth() const
	{
		return m_cellsIndexer.width + 1;
	}

	// Get the height of this grid in number of voxels.
	uint32 GetHeight() const
	{
		return m_cellsIndexer.height + 1;
	}

	// Get the depth of this grid in number of voxels.
	uint32 GetDepth() const
	{
		return m_cellsIndexer.depth + 1;
	}

	// Get the raw pointer to the voxel data.
	const T* GetVoxelData() const
	{
		return m_voxels;
	}

	// Get the number of voxels.
	uint32 GetVoxelCount() const
	{
		return m_voxelCount;
	}

	// Set the voxel value at the given voxel index.
	void SetVoxel(uint32 voxelIndex, const T& voxelValue)
	{
		B3_ASSERT(voxelIndex < m_voxelCount);
		m_voxels[voxelIndex] = voxelValue;
	}

	// Get the voxel value at the given voxel index.
	const T& GetVoxel(uint32 voxelIndex) const
	{
		B3_ASSERT(voxelIndex < m_voxelCount);
		return m_voxels[voxelIndex];
	}

	// Set the voxel value at the given 3D voxel index.
	void SetVoxel(const b3Index3D& voxelIndex, const T& voxelValue)
	{
		B3_ASSERT(Contains(voxelIndex));
		uint32 index1D = GetIndexInVoxelsArray(voxelIndex);
		B3_ASSERT(index1D < m_voxelCount);
		m_voxels[index1D] = voxelValue;
	}

	// Get the voxel value at the given 3D voxel index.
	const T& GetVoxel(const b3Index3D& voxelIndex) const
	{
		B3_ASSERT(Contains(voxelIndex));
		uint32 index1D = GetIndexInVoxelsArray(voxelIndex);
		B3_ASSERT(index1D < m_voxelCount);
		return m_voxels[index1D];
	}

	// Get the 3D position of the voxel at the given voxel index.
	b3Vec3 GetVoxelPosition(const b3Index3D& voxelIndex) const
	{
		B3_ASSERT(Contains(voxelIndex));
		return m_cellsIndexer.GetCellAABB(voxelIndex).lowerBound;
	}

	// Return an interpolated voxel value at the given point.
	// The point must be inside this grid. Call Contains() to verify if the point is inside the grid.
	T Sample(const b3Vec3& point) const
	{
		b3Index3D cellIndex = m_cellsIndexer.GetCellIndex(point);
		B3_ASSERT(m_cellsIndexer.Contains(cellIndex));

		b3AABB cellAABB = GetCellAABB(cellIndex);
		b3Vec3 relPoint = cellAABB.GetRelativePosition(point);

		T cellVoxels[8];
		GetCellVoxels(cellVoxels, cellIndex);

		return InterpolateVoxelValue(relPoint, cellVoxels);
	}

	// Return an interpolated gradient at the given point.
	// The point must be inside this grid. Call Contains() to verify if the point is inside the grid.
	// Note: If the gradient is a surface normal you must normalize the gradient vector.
	b3Vec3 SampleGradient(const b3Vec3& point) const
	{
		b3Index3D cellIndex = m_cellsIndexer.GetCellIndex(point);
		B3_ASSERT(m_cellsIndexer.Contains(cellIndex));

		b3AABB cellAABB = GetCellAABB(cellIndex);
		b3Vec3 relPoint = cellAABB.GetRelativePosition(point);

		T cellVoxels[8];
		GetCellVoxels(cellVoxels, cellIndex);

		b3Vec3 gradient;
		gradient.x = InterpolateVoxelValue(b3Vec3(scalar(1), relPoint.y, relPoint.z), cellVoxels) - InterpolateVoxelValue(b3Vec3(scalar(0), relPoint.y, relPoint.z), cellVoxels);
		gradient.y = InterpolateVoxelValue(b3Vec3(relPoint.x, scalar(1), relPoint.z), cellVoxels) - InterpolateVoxelValue(b3Vec3(relPoint.x, scalar(0), relPoint.z), cellVoxels);
		gradient.z = InterpolateVoxelValue(b3Vec3(relPoint.x, relPoint.y, scalar(1)), cellVoxels) - InterpolateVoxelValue(b3Vec3(relPoint.x, relPoint.y, scalar(0)), cellVoxels);
		return gradient;
	}

	// Get the width of this grid in number of cells.
	uint32 GetWidthInCells() const
	{
		return m_cellsIndexer.width;
	}
	
	// Get the height of this grid in number of cells.
	uint32 GetHeightInCells() const
	{
		return m_cellsIndexer.height;
	}
	
	// Get the depth of this grid in number of cells.
	uint32 GetDepthInCells() const
	{
		return m_cellsIndexer.depth;
	}
	
	// Get one cell (8 voxels) of this grid.
	void GetCellVoxels(T voxels[8], const b3Index3D& cellIndex) const
	{
		B3_ASSERT(ContainsCell(cellIndex));

		b3Index3D::ValueType i = cellIndex.i;
		b3Index3D::ValueType j = cellIndex.j;
		b3Index3D::ValueType k = cellIndex.k;

		voxels[0] = GetVoxel(b3Index3D(i, j, k));
		voxels[1] = GetVoxel(b3Index3D(i, j, k + 1));
		voxels[2] = GetVoxel(b3Index3D(i, j + 1, k));
		voxels[3] = GetVoxel(b3Index3D(i, j + 1, k + 1));
		voxels[4] = GetVoxel(b3Index3D(i + 1, j, k));
		voxels[5] = GetVoxel(b3Index3D(i + 1, j, k + 1));
		voxels[6] = GetVoxel(b3Index3D(i + 1, j + 1, k));
		voxels[7] = GetVoxel(b3Index3D(i + 1, j + 1, k + 1));
	}

	// Get he AABB of the cell specified by cellIndex.
	b3AABB GetCellAABB(const b3Index3D& cellIndex) const
	{
		return m_cellsIndexer.GetCellAABB(cellIndex);
	}

	// Get the index of the cell where point lies in.
	b3Index3D GetCellIndexOfPoint(const b3Vec3& point) const
	{
		return m_cellsIndexer.GetCellIndex(point);
	}

	// Get the bounding box for this grid.
	const b3AABB& GetAABB() const
	{
		return m_cellsIndexer.aabb;
	}

	// Does the given cell index point to a cell that is logically inside this grid?
	bool ContainsCell(const b3Index3D& cellIndex) const
	{
		return m_cellsIndexer.Contains(cellIndex);
	}

	// Does the given voxel index points to a voxel that is logically inside this grid?
	bool Contains(const b3Index3D& voxelIndex) const
	{
		return
			voxelIndex.i >= b3Index3D::ValueType(0) && voxelIndex.i < b3Index3D::ValueType(GetWidth()) &&
			voxelIndex.j >= b3Index3D::ValueType(0) && voxelIndex.j < b3Index3D::ValueType(GetHeight()) &&
			voxelIndex.k >= b3Index3D::ValueType(0) && voxelIndex.k < b3Index3D::ValueType(GetDepth());
	}

	 // Is the given point inside the AABB of this grid?
	bool Contains(const b3Vec3& point) const
	{
		return ContainsCell(GetCellIndexOfPoint(point));
	}
private:
	// Converts a given 3D index to a 1D index to be used in the voxels array.
	uint32 GetIndexInVoxelsArray(const b3Index3D& index) const
	{
		return uint32(index.GetOneDimensionalIndex(GetWidth(), GetHeight()));
	}

	// Trilinear interpolation given relative point inside the cell AABB and 8 voxels around the point.
	// Based on http://en.wikipedia.org/wiki/Trilinear_interpolation
	T InterpolateVoxelValue(const b3Vec3& relativePointInCell, const T voxels[8]) const
	{
		scalar x = b3Clamp(relativePointInCell.x, scalar(0), scalar(1));
		scalar y = b3Clamp(relativePointInCell.y, scalar(0), scalar(1));
		scalar z = b3Clamp(relativePointInCell.z, scalar(0), scalar(1));

		// x interpolation:
		scalar c00 = b3LinearInterpolation(x, voxels[0], voxels[4]);
		scalar c10 = b3LinearInterpolation(x, voxels[2], voxels[6]);
		scalar c01 = b3LinearInterpolation(x, voxels[1], voxels[5]);
		scalar c11 = b3LinearInterpolation(x, voxels[3], voxels[7]);

		// y interpolation:
		scalar c0 = b3LinearInterpolation(y, c00, c10);
		scalar c1 = b3LinearInterpolation(y, c01, c11);

		// z interpolation:
		return b3LinearInterpolation(z, c0, c1);
	}

	// Cells indexer.
	b3RegularGridIndexer m_cellsIndexer;

	// The voxel data. Owned by this class.
	T* m_voxels;

	// The voxel count.
	uint32 m_voxelCount;
};

using b3ScalarVoxelGrid = b3VoxelGrid<scalar>;

#endif

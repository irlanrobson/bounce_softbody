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

#ifndef B3_SDF_H
#define B3_SDF_H

#include <bounce_softbody/collision/geometry/voxel_grid.h>

struct b3Mesh;

// This binds a triangle mesh to a signed distance field (SDF).
class b3SDF
{
public:
	// Default ctor.
	b3SDF() : m_mesh(nullptr) { }

	// Get the associated mesh.
	const b3Mesh* GetMesh() { return m_mesh; }
	const b3Mesh* GetMesh() const { return m_mesh; }

	// Get the associated voxel grid.
	const b3ScalarVoxelGrid& GetVoxelGrid() const { return m_voxelGrid; }

	// Create the signed distance field from a given mesh and cell size.    
	// This is a very slow operation.
	void Create(const b3Mesh* mesh, const b3Vec3& cellSize, scalar aabbExtension = scalar(1));

	// Return the AABB of the voxel grid.
	const b3AABB& GetAABB() const { return m_voxelGrid.GetAABB(); }

	// Check if the given point is inside this grid AABB.
	bool Contains(const b3Vec3& point) const
	{
		return m_voxelGrid.Contains(point);
	}

	// Return the signed distance from a given point to the mesh.
	// The point must be inside the grid AABB. Check if the point is inside 
	// using Contains().
	scalar GetDistance(const b3Vec3& point) const
	{
		B3_ASSERT(m_voxelGrid.Contains(point));
		return m_voxelGrid.Sample(point);
	}

	// Return the surface normal of a given point to the mesh.
	// The point must be inside the grid AABB. Check if the point is inside 
	// using Contains().
	b3Vec3 GetSurfaceNormal(const b3Vec3& point) const
	{
		B3_ASSERT(m_voxelGrid.Contains(point));
		b3Vec3 gradient = m_voxelGrid.SampleGradient(point);
		return b3Normalize(gradient);
	}
protected:
	// Compute the signed distances and associate them to voxel grid. 
	// This is very ineffective.
	// Use AABB tree to faster queries?
	void ComputeDistances();

	// The mesh pointer.
	const b3Mesh* m_mesh;

	// Voxel grid containing point distances.
	b3ScalarVoxelGrid m_voxelGrid;
};

#endif
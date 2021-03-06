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

// For a given triangle mesh, implements the abstraction called "distance field" (aka "distance volume" or "distance function").
// It provides very fast triangle mesh distance queries for 3D points. It returns negative distances if the point is inside the mesh volume.
struct b3SDF
{
	const b3Mesh* mesh = nullptr;
	b3ScalarVoxelGrid voxelGrid;

	// Get the AABB of the voxel grid.
	const b3AABB& GetAABB() const;

	// Check if the given point is inside the voxel grid.
	bool Contains(const b3Vec3& point) const;

	// Return the signed distance from a given point to the mesh. Distances are negative for internal points.
	// The point must be inside the voxel grid. Check if the point is inside the voxel grid  
	// using Contains().
	scalar Distance(const b3Vec3& point) const;

	// Return the outward pointing normal of a given point to the mesh.
	// The point must be inside the voxel grid. Check if the point is inside the voxel grid 
	// using Contains().
	b3Vec3 Normal(const b3Vec3& point) const;
};

// Build the signed distance field from a given mesh, cell size, and an extension value that tells how much the mesh AABB should be extended by. 
// The mesh is assumed to be consistent. This code doesn't check mesh consistency. 
// Currently this is very ineffective. Consider saving an instance of the SDF object after building it. 
void b3BuildSDF(b3SDF* sdf, const b3Mesh* mesh, const b3Vec3& cellSize, scalar aabbVolumeExtension = scalar(1));

inline const b3AABB& b3SDF::GetAABB() const 
{ 
	return voxelGrid.GetAABB(); 
}

inline bool b3SDF::Contains(const b3Vec3& point) const
{
	return voxelGrid.Contains(point);
}

inline scalar b3SDF::Distance(const b3Vec3& point) const
{
	return voxelGrid.Sample(point);
}

inline b3Vec3 b3SDF::Normal(const b3Vec3& point) const
{
	b3Vec3 gradient = voxelGrid.SampleGradient(point);
	return b3Normalize(gradient);
}

#endif

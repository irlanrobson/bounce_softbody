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

#include <bounce_softbody/collision/geometry/sdf.h>
#include <bounce_softbody/collision/geometry/mesh.h>
#include <bounce_softbody/collision/geometry/geometry.h>
#include <bounce_softbody/collision/shapes/triangle_shape.h>

// Based on https://github.com/oprogramadorreal/vize's TriangleMeshDistanceFieldBuilder.

static bool b3IsPointInsideMesh(const b3Mesh* mesh, const b3Vec3& point, scalar farDistance)
{
	const auto isInside = [&mesh, &point](const b3Vec3& farPoint) -> bool 
	{
		uint32 counter = 0;

		for (uint32 i = 0; i < mesh->triangleCount; ++i)
		{
			b3Triangle* triangle = mesh->triangles + i;

			b3TriangleShape triangleShape;
			triangleShape.m_vertex1 = mesh->vertices[triangle->v1];
			triangleShape.m_vertex2 = mesh->vertices[triangle->v2];
			triangleShape.m_vertex3 = mesh->vertices[triangle->v3];

			b3RayCastInput input;
			input.p1 = point;
			input.p2 = farPoint;
			input.maxFraction = scalar(1);

			b3RayCastOutput output;
			if (triangleShape.RayCast(&output, input))
			{
				++counter;
			}
		}

		// If number of intersections is odd, point is inside.
		return (counter % 2) != 0;
	};

	// Test different directions to be sure.
	const b3Vec3 farPoints[5] =
	{
		b3Vec3(farDistance, 0, 0),
		b3Vec3(0, farDistance, 0),
		b3Vec3(0, 0, farDistance),
		b3Vec3(farDistance, 0, farDistance),
		b3Vec3(-farDistance, farDistance, 0)
	};

	uint32 inCount = 0;
	
	for (uint32 i = 0; i < 5 && inCount < 3; ++i) 
	{
		if (isInside(farPoints[i])) 
		{
			++inCount;
		}
	}

	return inCount >= 3;
}

static scalar b3ComputeDistance(const b3Mesh* mesh, const b3Vec3& point, scalar farDistance)
{
	scalar closestDistanceSquared = B3_MAX_SCALAR;
	b3Vec3 closestPoint(0, 0, 0);
	uint32 closestTriangle = 0;

	for (uint32 i = 0; i < mesh->triangleCount; ++i)
	{
		b3Triangle* triangle = mesh->triangles + i;

		b3Vec3 A = mesh->vertices[triangle->v1];
		b3Vec3 B = mesh->vertices[triangle->v2];
		b3Vec3 C = mesh->vertices[triangle->v3];

		b3Vec3 pointOnTriangle = b3ClosestPointOnTriangle(A, B, C, point);
		scalar distanceSquared = b3DistanceSquared(pointOnTriangle, point);

		if (distanceSquared < closestDistanceSquared)
		{
			closestTriangle = i;
			closestPoint = pointOnTriangle;
			closestDistanceSquared = distanceSquared;
		}
	}

	scalar closestDistance = b3Sqrt(closestDistanceSquared);

	if (b3IsPointInsideMesh(mesh, point, farDistance))
	{
		// Inside distances are negative.
		closestDistance *= scalar(-1);
	}

	return closestDistance;
}

static void b3ComputeDistances(b3SDF* sdf)
{
	const b3Mesh* mesh = sdf->mesh;
	b3ScalarVoxelGrid& voxelGrid = sdf->voxelGrid;

	scalar farDistance = voxelGrid.GetAABB().GetVolume();

	uint32 count = 0;
	scalar lastProgress = scalar(0);

	const auto updateProgressBar = [&count, &lastProgress, &voxelGrid]()
	{
		scalar progress = scalar(100) * (scalar(++count) / scalar(voxelGrid.GetVoxelCount()));

		if ((progress - lastProgress) >= scalar(1))
		{
			b3Log("[b3BuildSDF] Computing distances... %.0f%% - %d/%d\n", progress, count, voxelGrid.GetVoxelCount());
			lastProgress = progress;
		}
	};

	for (uint32 xIdx = 0; xIdx < voxelGrid.GetWidth(); ++xIdx)
	{
		for (uint32 yIdx = 0; yIdx < voxelGrid.GetHeight(); ++yIdx)
		{
			for (uint32 zIdx = 0; zIdx < voxelGrid.GetDepth(); ++zIdx)
			{
				b3Index3D voxelIndex = b3Index3D(xIdx, yIdx, zIdx);
				b3Vec3 voxelPosition = voxelGrid.GetVoxelPosition(voxelIndex);
				scalar distance = b3ComputeDistance(mesh, voxelPosition, farDistance);

				voxelGrid.SetVoxel(voxelIndex, distance);

				updateProgressBar();
			}
		}
	}
}

void b3BuildSDF(b3SDF* sdf, const b3Mesh* mesh, const b3Vec3& cellSize, scalar aabbVolumeExtension)
{
	B3_ASSERT(sdf->mesh == nullptr);
	sdf->mesh = mesh;

	b3AABB aabb = mesh->ComputeAABB();
	aabb.Extend(aabbVolumeExtension);
	
	b3Vec3 aabbSize = aabb.GetDimensions();

	uint32 widthInCells = uint32(std::ceil(aabbSize.x / cellSize.x));
	uint32 heightInCells = uint32(std::ceil(aabbSize.y / cellSize.y));
	uint32 depthInCells = uint32(std::ceil(aabbSize.z / cellSize.z));

	// Create voxel grid.
	sdf->voxelGrid.Create(aabb, widthInCells + 1, heightInCells + 1, depthInCells + 1);

	// Compute distances.
	b3ComputeDistances(sdf);
}

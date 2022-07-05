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

static b3Vec3 b3ClosestPointOnTriangle(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, 
	const b3Vec3& Q)
{
	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		return A;
	}

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		return B;
	}

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		return C;
	}

	// Test edge regions		
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);
	
	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wABC[3] * wABC[2] <= scalar(0))
	{
		B3_ASSERT(wAB[2] > scalar(0));
		return (wAB[0] * A + wAB[1] * B) / wAB[2];
	}

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && wABC[3] * wABC[0] <= scalar(0))
	{
		B3_ASSERT(wBC[2] > scalar(0));
		return (wBC[0] * B + wBC[1] * C) / wBC[2];
	}

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && wABC[3] * wABC[1] <= scalar(0))
	{
		B3_ASSERT(wCA[2] > scalar(0));
		return (wCA[0] * C + wCA[1] * A) / wCA[2];
	}

	if (wABC[3] == scalar(0))
	{
		// Return the closest point on largest segment?
		return A;
	}

	// R ABC/ACB
	B3_ASSERT(wABC[3] > scalar(0));
	return (wABC[0] * A + wABC[1] * B + wABC[2] * C) / wABC[3];
}

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

static scalar b3SignedDistance(const b3Mesh* mesh, const b3Vec3& point, scalar farDistance)
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

void b3SDF::Build(const b3Mesh* mesh, const b3Vec3& cellSize, scalar aabbVolumeExtension)
{
	B3_ASSERT(m_mesh == nullptr);
	m_mesh = mesh;

	b3AABB aabb = m_mesh->ComputeAABB();
	aabb.Extend(aabbVolumeExtension);
	
	b3Vec3 aabbSize = aabb.GetDimensions();

	uint32 widthInCells = uint32(std::ceil(aabbSize.x / cellSize.x));
	uint32 heightInCells = uint32(std::ceil(aabbSize.y / cellSize.y));
	uint32 depthInCells = uint32(std::ceil(aabbSize.z / cellSize.z));

	// Create voxel grid.
	m_voxelGrid.Create(aabb, widthInCells + 1, heightInCells + 1, depthInCells + 1);

	// Compute distances.
	ComputeDistances();
}

void b3SDF::ComputeDistances()
{
	scalar farDistance = m_voxelGrid.GetAABB().GetVolume();

	uint32 count = 0;
	scalar lastProgress = scalar(0);

	const auto updateProgressBar = [&count, &lastProgress, this]() 
	{
		scalar progress = scalar(100) * (scalar(++count) / scalar(m_voxelGrid.GetVoxelCount()));

		if ((progress - lastProgress) >= scalar(1)) 
		{
			b3Log("[b3SDF] Calculating distances... %.0f%% - %d/%d\n", progress, count, m_voxelGrid.GetVoxelCount());
			lastProgress = progress;
		}
	};

	for (uint32 xIdx = 0; xIdx < m_voxelGrid.GetWidth(); ++xIdx) 
	{
		for (uint32 yIdx = 0; yIdx < m_voxelGrid.GetHeight(); ++yIdx) 
		{
			for (uint32 zIdx = 0; zIdx < m_voxelGrid.GetDepth(); ++zIdx) 
			{
				b3Index3D voxelIndex = b3Index3D(xIdx, yIdx, zIdx);
				b3Vec3 voxelPosition = m_voxelGrid.GetVoxelPosition(voxelIndex);
				scalar signedDistance = b3SignedDistance(m_mesh, voxelPosition, farDistance);

				m_voxelGrid.SetVoxel(voxelIndex, signedDistance);

				updateProgressBar();
			}
		}
	}
}

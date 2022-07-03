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

#include <bounce_softbody/collision/shapes/sdf_shape.h>
#include <bounce_softbody/collision/geometry/sdf.h>
#include <bounce_softbody/collision/geometry/mesh.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

b3SDFShape::b3SDFShape()
{
	m_type = e_sdf;
	m_radius = scalar(0);
	m_sdf = nullptr;
}

b3Shape* b3SDFShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3SDFShape));
	b3SDFShape* clone = new (mem)b3SDFShape;
	*clone = *this;
	return clone;
}

b3AABB b3SDFShape::ComputeAABB() const
{
	return m_sdf->GetAABB();
}

bool b3SDFShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 center = sphere.vertex;
	scalar radius = sphere.radius + m_radius;

	// The sphere center must be contained in the SDF's AABB.
	if (m_sdf->Contains(center) == false)
	{
		return false;
	}

	scalar distance = m_sdf->GetDistance(center);
	if (distance > radius)
	{
		return false;
	}
	
	b3Vec3 normal = m_sdf->GetSurfaceNormal(center);
	b3Vec3 closestPoint = center - distance * normal;

	manifold->point = closestPoint;
	manifold->normal = normal;
	return true;
}

void b3SDFShape::Draw(b3Draw* draw) const
{
	const b3Mesh* mesh = m_sdf->GetMesh();
	for (uint32 i = 0; i < mesh->triangleCount; ++i)
	{
		b3Triangle* triangle = mesh->triangles + i;

		b3Vec3 A = mesh->vertices[triangle->v1];
		b3Vec3 B = mesh->vertices[triangle->v2];
		b3Vec3 C = mesh->vertices[triangle->v3];

		draw->DrawTriangle(A, B, C, b3Color_black);

		b3Vec3 N = b3Cross(B - A, C - A);
		N.Normalize();

		draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
		draw->DrawSolidTriangle(-N, A, C, B, b3Color_gray);
	}

	const b3ScalarVoxelGrid& voxelGrid = m_sdf->GetVoxelGrid();
	draw->DrawAABB(voxelGrid.GetAABB(), b3Color_pink);
	
	for (uint32 i = 0; i < voxelGrid.GetWidthInCells(); ++i)
	{
		for (uint32 j = 0; j < voxelGrid.GetHeightInCells(); ++j)
		{
			for (uint32 k = 0; k < voxelGrid.GetDepthInCells(); ++k)
			{
				b3Index3D cellIndex = b3Index3D(i, j, k);
				b3AABB cellAABB = voxelGrid.GetCellAABB(cellIndex);
				
				draw->DrawAABB(cellAABB, b3Color_pink);

				b3Vec3 cellCenter = cellAABB.GetCenter();
				scalar distance = voxelGrid.Sample(cellCenter);

				if (distance <= scalar(0))
				{
					draw->DrawPoint(cellCenter, scalar(4), b3Color_red, false);
				}
				else
				{
					draw->DrawPoint(cellCenter, scalar(4), b3Color_green, false);
				}
			}
		}
	}
}
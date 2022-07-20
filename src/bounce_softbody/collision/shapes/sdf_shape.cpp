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
	m_xf.SetIdentity();
	m_invert = false;
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
	b3AABB aabb = m_sdf->GetAABB();
	aabb.Transform(m_xf);
	aabb.Extend(m_radius);
	return aabb;
}

bool b3SDFShape::Collide(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	// Convert the sphere center to local space of SDF.
	b3Vec3 point = b3MulT(m_xf, sphere.vertex);
	scalar radius = sphere.radius + m_radius;

	// The sphere center must be contained in the SDF's AABB.
	if (m_sdf->Contains(point) == false)
	{
		return false;
	}

	scalar distance = m_sdf->Distance(point);	
	if (m_invert)
	{
		distance *= scalar(-1);
	}
	
	if (distance > radius)
	{
		return false;
	}
	
	b3Vec3 normal = m_sdf->Normal(point);
	if (m_invert)
	{
		normal *= scalar(-1);
	}

	b3Vec3 surfaceNormal = b3Mul(m_xf.rotation, normal);
	b3Vec3 surfacePoint = sphere.vertex - distance * surfaceNormal;

	manifold->point = surfacePoint;
	manifold->normal = surfaceNormal;
	return true;
}

void b3SDFShape::Draw(b3Draw* draw) const
{
	const b3Mesh* mesh = m_sdf->mesh;
	for (uint32 i = 0; i < mesh->triangleCount; ++i)
	{
		b3Triangle* triangle = mesh->triangles + i;

		b3Vec3 A = m_xf * mesh->vertices[triangle->v1];
		b3Vec3 B = m_xf * mesh->vertices[triangle->v2];
		b3Vec3 C = m_xf * mesh->vertices[triangle->v3];

		draw->DrawTriangle(A, B, C, b3Color_black);

		b3Vec3 N = b3Cross(B - A, C - A);
		N.Normalize();

		if (m_invert == false)
		{
			draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
		}

		b3Vec3 center = (A + B + C) / scalar(3);
		draw->DrawSegment(center, center + scalar(1) * N, b3Color_pink);
	}
	
	b3AABB aabb = ComputeAABB();
	draw->DrawAABB(aabb, b3Color_pink);
	
	const b3ScalarVoxelGrid& voxelGrid = m_sdf->voxelGrid;
	for (uint32 i = 0; i < voxelGrid.GetWidth(); ++i)
	{
		for (uint32 j = 0; j < voxelGrid.GetHeight(); ++j)
		{
			for (uint32 k = 0; k < voxelGrid.GetDepth(); ++k)
			{
				b3Index3D voxelIndex = b3Index3D(i, j, k);
				scalar voxelValue = voxelGrid.GetVoxel(voxelIndex);
				b3Vec3 voxelPosition = voxelGrid.GetVoxelPosition(voxelIndex);

				b3Vec3 position = b3Mul(m_xf, voxelPosition);
				if (voxelValue <= scalar(0))
				{
					draw->DrawPoint(position, scalar(2), b3Color_red, false);
				}
				else
				{
					//draw->DrawPoint(position, scalar(2), b3Color_green, false);
				}
			}
		}
	}
}

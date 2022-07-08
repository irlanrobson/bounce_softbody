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

#include <bounce_softbody/collision/shapes/mesh_shape.h>
#include <bounce_softbody/collision/geometry/mesh.h>
#include <bounce_softbody/collision/shapes/triangle_shape.h>
#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/draw.h>

#define B3_NULL_TRIANGLE B3_MAX_U32

b3MeshShape::b3MeshShape() 
{
	m_type = e_mesh;
	m_radius = scalar(0);
	m_mesh = nullptr;
	m_xf.SetIdentity();
	m_scale.Set(scalar(1), scalar(1), scalar(1));
}

b3Shape* b3MeshShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3MeshShape));
	b3MeshShape* clone = new (mem)b3MeshShape;
	*clone = *this;
	return clone;
}

b3AABB b3MeshShape::ComputeAABB() const 
{
	b3AABB aabb;
	aabb.Compute(m_mesh->vertices, m_mesh->vertexCount, m_scale, m_xf);
	aabb.Extend(m_radius);
	return aabb;
}

// This contains the closest point on the mesh to the given sphere.
struct b3MeshShapeQueryWrapper
{
	bool Report(uint32 nodeId)
	{
		uint32 index = meshShape->m_mesh->tree.GetIndex(nodeId);
		
		// Get the child triangle in world space.
		b3TriangleShape triangle;
		meshShape->GetChildTriangle(&triangle, index);

		// Get the closest point on the triangle to the sphere center in world space.
		b3SphereManifold manifold;
		if (triangle.Collide(&manifold, sphere))
		{
			scalar dd = b3DistanceSquared(sphere.vertex, manifold.point);
			if (dd < dd0)
			{
				dd0 = dd;
				manifold0 = manifold;
				triangle0 = index;
			}
		}

		// Keep looking for overlaps.
		return true;
	}

	const b3MeshShape* meshShape;
	b3Sphere sphere;
	scalar dd0;
	b3SphereManifold manifold0;
	uint32 triangle0;
};

bool b3MeshShape::Collide(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	b3MeshShapeQueryWrapper wrapper;
	wrapper.meshShape = this;
	wrapper.sphere = sphere;
	wrapper.dd0 = B3_MAX_SCALAR;
	wrapper.triangle0 = B3_NULL_TRIANGLE;

	B3_ASSERT(m_scale.x != scalar(0));
	B3_ASSERT(m_scale.y != scalar(0));
	B3_ASSERT(m_scale.z != scalar(0));

	b3Vec3 invScale;
	invScale.x = scalar(1) / m_scale.x;
	invScale.y = scalar(1) / m_scale.y;
	invScale.z = scalar(1) / m_scale.z;

	// Transform the sphere center from world frame to unscaled tree frame.
	// Take the mesh radius into account.
	b3Vec3 center = b3Mul(invScale, b3MulT(m_xf, sphere.vertex));
	scalar radius = sphere.radius + m_radius;

	// Local sphere AABB.
	b3AABB treeAABB(center, radius);
	
	// Run the query.
	m_mesh->tree.Query(&wrapper, treeAABB);

	if (wrapper.triangle0 != B3_NULL_TRIANGLE)
	{
		manifold->point = wrapper.manifold0.point;
		manifold->normal = wrapper.manifold0.normal;
		return true;
	}

	return false;
}

void b3MeshShape::GetChildTriangle(b3TriangleShape* triangleShape, uint32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	b3Triangle* triangle = m_mesh->triangles + index;
	
	uint32 u1 = triangle->u1;
	uint32 u2 = triangle->u2;
	uint32 u3 = triangle->u3;

	b3Vec3 v1 = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v1]);
	b3Vec3 v2 = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v2]);
	b3Vec3 v3 = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v3]);

	triangleShape->m_vertex1 = v1;
	triangleShape->m_vertex2 = v2;
	triangleShape->m_vertex3 = v3;
	triangleShape->m_radius = m_radius;

	if (u1 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE1Vertex = true;
		triangleShape->m_e1Vertex = m_xf * b3Mul(m_scale, m_mesh->vertices[u1]);
	}

	if (u2 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE2Vertex = true;
		triangleShape->m_e2Vertex = m_xf * b3Mul(m_scale, m_mesh->vertices[u2]);
	}

	if (u3 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE3Vertex = true;
		triangleShape->m_e3Vertex = m_xf * b3Mul(m_scale, m_mesh->vertices[u3]);
	}
}

void b3MeshShape::Draw(b3Draw* draw) const
{
	for (uint32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3Triangle* triangle = m_mesh->triangles + i;

		b3Vec3 A = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v1]);
		b3Vec3 B = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v2]);
		b3Vec3 C = m_xf * b3Mul(m_scale, m_mesh->vertices[triangle->v3]);

		draw->DrawTriangle(A, B, C, b3Color_black);

		b3Vec3 N = b3Cross(B - A, C - A);
		N.Normalize();
		
		draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
		draw->DrawSolidTriangle(-N, A, C, B, b3Color_gray);
	}
}

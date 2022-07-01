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

#include <bounce_softbody/collision/geometry/mesh.h>

void b3Mesh::BuildTree()
{
	// This function must be called once.
	b3AABB* aabbs = (b3AABB*)b3Alloc(triangleCount * sizeof(b3AABB));
	for (uint32 i = 0; i < triangleCount; ++i)
	{
		aabbs[i] = GetTriangleAABB(i);
	}

	// Build the tree. 
	tree.Build(aabbs, triangleCount);

	b3Free(aabbs);
}

void b3Mesh::BuildAdjacency()
{
	for (uint32 i1 = 0; i1 < triangleCount; ++i1)
	{
		b3Triangle* t1 = triangles + i1;

		for (uint32 j1 = 0; j1 < 3; ++j1)
		{
			uint32 k1 = j1 + 1 < 3 ? j1 + 1 : 0;

			uint32 t1v1 = t1->GetVertex(j1);
			uint32 t1v2 = t1->GetVertex(k1);

			uint32& u1 = t1->GetWingVertex(j1);

			// Mark edge as open.
			u1 = B3_NULL_VERTEX;

			// Search the first adjacent triangle. 
			for (uint32 i2 = 0; i2 < triangleCount; ++i2)
			{
				if (i1 == i2)
				{
					continue;
				}

				b3Triangle* t2 = triangles + i2;

				for (uint32 j2 = 0; j2 < 3; ++j2)
				{
					uint32 k2 = j2 + 1 < 3 ? j2 + 1 : 0;

					uint32 t2v1 = t2->GetVertex(j2);
					uint32 t2v2 = t2->GetVertex(k2);

					if (t1v1 == t2v2 && t1v2 == t2v1)
					{
						// The triangles are adjacent.

						// Non-shared vertex on adjacent triangle. 
						uint32 n2 = k2 + 1 < 3 ? k2 + 1 : 0;

						u1 = t2->GetVertex(n2);

						break;
					}
				}

				if (u1 != B3_NULL_VERTEX)
				{
					break;
				}
			}
		}
	}
}

void b3Mesh::Scale(const b3Vec3& scale)
{
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(scale, vertices[i]);
	}
}

void b3Mesh::Rotate(const b3Quat& rotation)
{
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(rotation, vertices[i]);
	}
}

void b3Mesh::Translate(const b3Vec3& translation)
{
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] += translation;
	}
}

void b3Mesh::Transform(const b3Transform& xf, const b3Vec3& scale)
{
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(xf, b3Mul(scale, vertices[i]));
	}
}
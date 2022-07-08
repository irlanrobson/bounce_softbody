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

#ifndef B3_CYLINDER_MESH_H
#define B3_CYLINDER_MESH_H

#include <bounce_softbody/collision/geometry/mesh.h>

// A (H + 1) x (W + 1) cylinder mesh stored in row-major order.
// v(i, j) = i * (W + 1) + j
template<uint32 H = 1, uint32 W = 1>
struct b3CylinderMesh : public b3Mesh
{
	b3Vec3 cylinderVertices[(H + 1) * (W + 1)];
	b3Triangle cylinderTriangles[2 * H * W + 2 * (((W + 1) - 1) - 1)];

	b3CylinderMesh()
	{
		// Build vertices

		// Angular increment in range [0, 2*pi]
		scalar kPhiInc = scalar(2) * B3_PI / scalar(W);

		// Longitude increment in range [0, 1]
		scalar kYInc = scalar(1) / scalar(H);

		vertexCount = 0;
		for (uint32 i = 0; i < H + 1; ++i)
		{
			// Plane to cylindrical coordinates
			scalar y = scalar(i) * kYInc;

			for (uint32 j = 0; j < W + 1; ++j)
			{
				// Plane to cylindrical coordinates
				scalar phi = scalar(j) * kPhiInc;
				scalar cos_phi = cos(phi);
				scalar sin_phi = sin(phi);

				// Cylindrical to Cartesian coordinates		
				b3Vec3 p;
				p.x = cos_phi;
				p.y = y - scalar(0.5); // Centralize
				p.z = sin_phi;

				uint32 vertex = GetVertex(i, j);
				cylinderVertices[vertex] = p;
				++vertexCount;
			}
		}

		B3_ASSERT(vertexCount == (H + 1) * (W + 1));

		// Build triangles
		triangleCount = 0;
		for (uint32 i = 0; i < H; ++i)
		{
			for (uint32 j = 0; j < W; ++j)
			{
				// 1*|----|*4
				//   |----|
				// 2*|----|*3
				uint32 v1 = GetVertex(i, j);
				uint32 v2 = GetVertex(i + 1, j);
				uint32 v3 = GetVertex(i + 1, j + 1);
				uint32 v4 = GetVertex(i, j + 1);

				b3Triangle* t1 = cylinderTriangles + triangleCount++;
				t1->v1 = v1;
				t1->v2 = v2;
				t1->v3 = v3;

				t1->u1 = B3_NULL_VERTEX;
				t1->u2 = B3_NULL_VERTEX;
				t1->u3 = B3_NULL_VERTEX;

				b3Triangle* t2 = cylinderTriangles + triangleCount++;
				t2->v1 = v3;
				t2->v2 = v4;
				t2->v3 = v1;

				t2->u1 = B3_NULL_VERTEX;
				t2->u2 = B3_NULL_VERTEX;
				t2->u3 = B3_NULL_VERTEX;
			}
		}

		B3_ASSERT(triangleCount == 2 * H * W);

		// Lower circle
		uint32 i1 = 0;
		for (uint32 i2 = i1 + 1; i2 < (W + 1) - 1; ++i2)
		{
			uint32 i3 = i2 + 1;

			uint32 v1 = GetVertex(0, i1);
			uint32 v2 = GetVertex(0, i2);
			uint32 v3 = GetVertex(0, i3);
			
			b3Triangle* t = cylinderTriangles + triangleCount++;
			t->v1 = v1;
			t->v2 = v2;
			t->v3 = v3;

			t->u1 = B3_NULL_VERTEX;
			t->u2 = B3_NULL_VERTEX;
			t->u3 = B3_NULL_VERTEX;
		}

		// Upper circle
		i1 = 0;
		for (uint32 i2 = i1 + 1; i2 < (W + 1) - 1; ++i2)
		{
			uint32 i3 = i2 + 1;

			uint32 v1 = GetVertex(H, i1);
			uint32 v2 = GetVertex(H, i2);
			uint32 v3 = GetVertex(H, i3);

			// Flip order to ensure CCW
			b3Triangle* t = cylinderTriangles + triangleCount++;
			t->v1 = v3;
			t->v2 = v2;
			t->v3 = v1;

			t->u1 = B3_NULL_VERTEX;
			t->u2 = B3_NULL_VERTEX;
			t->u3 = B3_NULL_VERTEX;
		}

		B3_ASSERT(triangleCount == 2 * H * W + 2 * (((W + 1) - 1) - 1));

		vertices = cylinderVertices;
		triangles = cylinderTriangles;
	}

	uint32 GetVertex(uint32 i, uint32 j) const
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};

#endif
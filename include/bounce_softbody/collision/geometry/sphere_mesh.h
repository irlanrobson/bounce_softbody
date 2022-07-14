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

#ifndef B3_SPHERE_MESH_H
#define B3_SPHERE_MESH_H

#include <bounce_softbody/collision/geometry/mesh.h>

// A (H + 1) x (W + 1) sphere mesh stored in row-major order.
// v(i, j) = i * (W + 1) + j
template <uint32 H = 1, uint32 W = 1>
struct b3SphereMesh : public b3Mesh
{
	b3Vec3 sphereVertices[(H + 1) * (W + 1)];
	b3Triangle sphereTriangles[2 * H * W];

	// Set this grid to a W (width) per H (height) dimensioned grid centered at the origin and aligned
	// with the world x-z axes.
	b3SphereMesh()
	{
		// Build vertices

		// Latitude increment in range [0, pi]
		scalar kThetaInc = B3_PI / scalar(H);

		// Longitude increment in range [0, 2*pi]
		scalar kPhiInc = scalar(2) * B3_PI / scalar(W);

		vertexCount = 0;
		for (uint32 i = 0; i < H + 1; ++i)
		{
			// Plane to spherical coordinates
			scalar theta = scalar(i) * kThetaInc;
			scalar cos_theta = cos(theta);
			scalar sin_theta = sin(theta);

			for (uint32 j = 0; j < W + 1; ++j)
			{
				scalar phi = scalar(j) * kPhiInc;
				scalar cos_phi = cos(phi);
				scalar sin_phi = sin(phi);

				// Spherical to Cartesian coordinates		
				b3Vec3 p;
				p.x = sin_theta * sin_phi;
				p.y = cos_theta;
				p.z = sin_theta * cos_phi;

				uint32 vertex = GetVertex(i, j);
				sphereVertices[vertex] = p;
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

				b3Triangle* t1 = sphereTriangles + triangleCount++;
				t1->v1 = v1;
				t1->v2 = v2;
				t1->v3 = v3;
				
				t1->u1 = B3_NULL_VERTEX;
				t1->u2 = B3_NULL_VERTEX;
				t1->u3 = B3_NULL_VERTEX;

				b3Triangle* t2 = sphereTriangles + triangleCount++;
				t2->v1 = v3;
				t2->v2 = v4;
				t2->v3 = v1;
				
				t2->u1 = B3_NULL_VERTEX;
				t2->u2 = B3_NULL_VERTEX;
				t2->u3 = B3_NULL_VERTEX;
			}
		}

		B3_ASSERT(triangleCount == 2 * H * W);

		vertices = sphereVertices;
		triangles = sphereTriangles;
	}

	uint32 GetVertex(uint32 i, uint32 j) const
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};


#endif
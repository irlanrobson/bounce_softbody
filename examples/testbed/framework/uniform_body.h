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

#ifndef UNIFORM_BODY_H
#define UNIFORM_BODY_H

#include <bounce_softbody/bounce_softbody.h>

struct BodyMeshTriangle
{
	int v1, v2, v3;
};

struct BodyMeshTetrahedron
{
	int v1, v2, v3, v4;
};

struct BodyMesh
{
	int vertexCount;
	b3Vec3* vertices;
	int triangleCount;
	BodyMeshTriangle* triangles;
	int tetrahedronCount;
	BodyMeshTetrahedron* tetrahedrons;

	b3Vec3 GetVertexPosition(int index) const
	{
		return vertices[index];
	}

	BodyMeshTriangle GetTriangle(int index) const
	{
		return triangles[index];
	}

	BodyMeshTetrahedron GetTetrahedron(int index) const
	{
		return tetrahedrons[index];
	}

	void Scale(const b3Vec3& scale)
	{
		assert(scale.x != scalar(0));
		assert(scale.y != scalar(0));
		assert(scale.z != scalar(0));
		for (int i = 0; i < vertexCount; ++i)
		{
			vertices[i].x *= scale.x;
			vertices[i].y *= scale.y;
			vertices[i].z *= scale.z;
		}
	}

	void Translate(const b3Vec3& translation)
	{
		for (int i = 0; i < vertexCount; ++i)
		{
			vertices[i] += translation;
		}
	}
};

struct ClothDef
{
	ClothDef()
	{
		density = scalar(0.1);
		massDamping = scalar(0);
		thickness = scalar(0.0);
		friction = scalar(0.3);

		stretchingStiffness = scalar(1234);
		stretchStiffnessDamping = scalar(0);

		createElements = false;
		elementYoungModulus = scalar(500);
		elementShearModulus = scalar(500);
		elementPoissonRatio = scalar(0.3);
		elementStiffnessDamping = scalar(0);
	}

	scalar density;
	scalar massDamping;
	scalar thickness;
	scalar friction;

	const BodyMesh* mesh;

	scalar stretchingStiffness;
	scalar stretchStiffnessDamping;

	bool createElements;
	scalar elementYoungModulus;
	scalar elementShearModulus;
	scalar elementPoissonRatio;
	scalar elementStiffnessDamping;
};

struct TetDef
{
	TetDef()
	{
		density = scalar(0.1);
		massDamping = scalar(0);
		thickness = scalar(0.02);
		friction = scalar(0.3);
		elementYoungModulus = scalar(1000);
		elementPoissonRatio = scalar(0.3);
		elementStiffnessDamping = scalar(0);
	}
	
	scalar density;
	scalar massDamping;
	scalar thickness;
	scalar friction;

	const BodyMesh* mesh;

	scalar elementYoungModulus;
	scalar elementPoissonRatio;
	scalar elementStiffnessDamping;
};

// b3Body wrapper. 
class UniformBody : public b3Body
{
public:
	UniformBody();
	UniformBody(const ClothDef& def);
	UniformBody(const TetDef& def);
	~UniformBody();

	// Return particle associated with given mesh vertex.
	b3Particle* GetParticle(int index)
	{
		assert(index < m_mesh->vertexCount);
		return m_particles[index];
	}
private:
	const BodyMesh* m_mesh;
	b3Particle** m_particles;
};

template<int H = 1, int W = 1>
struct GridClothMesh : public BodyMesh
{
	b3Vec3 gridVertices[(H + 1) * (W + 1)];
	BodyMeshTriangle gridTriangles[2 * H * W];

	GridClothMesh()
	{
		// Generate vertices
		b3Vec3 center;
		center.x = 0.5f * float(W);
		center.y = 0;
		center.z = 0.5f * float(H);

		vertexCount = 0;
		vertices = gridVertices;
		for (int i = 0; i < H + 1; ++i)
		{
			for (int j = 0; j < W + 1; ++j)
			{
				int vertex = GetVertex(i, j);
				
				b3Vec3 position;
				position.x = j;
				position.y = 0.0f;
				position.z = i;
				
				position -= center;
				
				vertices[vertex] = position;
				
				++vertexCount;
			}
		}

		assert(vertexCount == (H + 1) * (W + 1));

		// Generate triangles
		triangleCount = 0;
		triangles = gridTriangles;
		for (int i = 0; i < H; ++i)
		{
			for (int j = 0; j < W; ++j)
			{
				// 1*|----|*4
				//   |----|
				// 2*|----|*3
				int v1 = GetVertex(i, j);
				int v2 = GetVertex(i + 1, j);
				int v3 = GetVertex(i + 1, j + 1);
				int v4 = GetVertex(i, j + 1);

				BodyMeshTriangle* t1 = triangles + triangleCount++;
				t1->v1 = v1;
				t1->v2 = v2;
				t1->v3 = v3;

				BodyMeshTriangle* t2 = triangles + triangleCount++;
				t2->v1 = v3;
				t2->v2 = v4;
				t2->v3 = v1;
			}
		}

		assert(triangleCount == 2 * H * W);
		
		tetrahedronCount = 0;
	}
	
	int GetRowVertexCount() const
	{
		return H + 1;
	}

	int GetColumnVertexCount() const
	{
		return W + 1;
	}
	
	int GetVertex(int i, int j)
	{
		assert(i < H + 1);
		assert(j < W + 1);
		return i * (W + 1) + j;
	}
};

template<int H = 1, int W = 1, int D = 1>
struct GridTetMesh : public BodyMesh
{
	b3Vec3 gridVertices[(H + 1) * (W + 1) * (D + 1)];
	BodyMeshTriangle gridTriangles[4 * H * W + 4 * H * D + 4 * W * D];
	BodyMeshTetrahedron gridTetrahedrons[5 * H * W * D];
	
	GridTetMesh()
	{
		// Generate vertices
		b3Vec3 center;
		center.x = 0.5f * (float)W;
		center.y = 0.5f * (float)H;
		center.z = 0.5f * (float)D;
		
		vertices = gridVertices;
		vertexCount = 0;
		for (int i = 0; i < H + 1; ++i)
		{
			for (int j = 0; j < W + 1; ++j)
			{
				for (int k = 0; k < D + 1; ++k)
				{
					int vertex = GetVertex(i, j, k);
					
					b3Vec3 position(j, i, k);
					
					position -= center;
					
					vertices[vertex] = position;
					++vertexCount;
				}
			}
		}
		
		// Generate triangles
		triangles = gridTriangles;
		triangleCount = 0;
		
		// x-y plane
		for (int i = 0; i < H; ++i)
		{
			for (int j = 0; j < W; ++j)
			{
				//     2*-----*3
				//     /|    /|
				//    / |   / |
				//   *-----*  |
				//   | 1*--|--*4
				//   | /   | /
				//   |/    |/
				//   *-----*
				{
					int v1 = GetVertex(i, j, 0);
					int v2 = GetVertex(i + 1, j, 0);
					int v3 = GetVertex(i + 1, j + 1, 0);
					int v4 = GetVertex(i, j + 1, 0);

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v3;

					++triangleCount;

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					int v1 = GetVertex(i, j, D);
					int v2 = GetVertex(i + 1, j, D);
					int v3 = GetVertex(i + 1, j + 1, D);
					int v4 = GetVertex(i, j + 1, D);

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v1;

					++triangleCount;

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		// y-z plane
		for (int i = 0; i < H; ++i)
		{
			for (int k = 0; k < D; ++k)
			{
				//      4*-----* 
				//     /|     /|
				//    / |    / |
				//   3*-----*  |
				//   |  1*--|--* 
				//   | /   | /
				//   |/    |/
				//   2*-----*
				{
					int v1 = GetVertex(i, 0, k);
					int v2 = GetVertex(i, 0, k + 1);
					int v3 = GetVertex(i + 1, 0, k + 1);
					int v4 = GetVertex(i + 1, 0, k);

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v3;

					++triangleCount;

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					int v1 = GetVertex(i, W, k);
					int v2 = GetVertex(i, W, k + 1);
					int v3 = GetVertex(i + 1, W, k + 1);
					int v4 = GetVertex(i + 1, W, k);

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v1;

					++triangleCount;

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		// x-z plane
		for (int j = 0; j < W; ++j)
		{
			for (int k = 0; k < D; ++k)
			{
				//      *-----* 
				//     /|     /|
				//    / |    / |
				//   *-----*   |
				//   |  1*----2* 
				//   | /   | /
				//   |/    |/
				//   4*-----3*
				{
					int v1 = GetVertex(0, j, k);
					int v2 = GetVertex(0, j + 1, k);
					int v3 = GetVertex(0, j + 1, k + 1);
					int v4 = GetVertex(0, j, k + 1);

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v3;

					++triangleCount;

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					int v1 = GetVertex(H, j, k);
					int v2 = GetVertex(H, j + 1, k);
					int v3 = GetVertex(H, j + 1, k + 1);
					int v4 = GetVertex(H, j, k + 1);

					triangles[triangleCount].v1 = v3;
					triangles[triangleCount].v2 = v2;
					triangles[triangleCount].v3 = v1;

					++triangleCount;

					triangles[triangleCount].v1 = v1;
					triangles[triangleCount].v2 = v4;
					triangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		assert(triangleCount == 4 * H * W + 4 * H * D + 4 * W * D);
		
		// Generate tetrahedrons
		tetrahedrons = gridTetrahedrons;
		tetrahedronCount = 0;
		
		for (int i = 0; i < H; ++i)
		{
			for (int j = 0; j < W; ++j)
			{
				for (int k = 0; k < D; ++k)
				{
					//     4*-----8* 
					//     /|     /|
					//    / |    / |
					//  3*-----7*  |
					//   | 1*--|--5*
					//   | /   |  /
					//   |/    | /
					//  2*-----6*
					int v1 = GetVertex(i, j, k);
					int v2 = GetVertex(i, j, k + 1);
					int v3 = GetVertex(i + 1, j, k + 1);
					int v4 = GetVertex(i + 1, j, k);

					int v5 = GetVertex(i, j + 1, k);
					int v6 = GetVertex(i, j + 1, k + 1);
					int v7 = GetVertex(i + 1, j + 1, k + 1);
					int v8 = GetVertex(i + 1, j + 1, k);

					// eliminate me
					if ((i + j + k) % 2 == 1)
					{
						tetrahedrons[tetrahedronCount].v1 = v2;
						tetrahedrons[tetrahedronCount].v2 = v6;
						tetrahedrons[tetrahedronCount].v3 = v7;
						tetrahedrons[tetrahedronCount].v4 = v5;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v5;
						tetrahedrons[tetrahedronCount].v2 = v7;
						tetrahedrons[tetrahedronCount].v3 = v4;
						tetrahedrons[tetrahedronCount].v4 = v8;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v2;
						tetrahedrons[tetrahedronCount].v2 = v4;
						tetrahedrons[tetrahedronCount].v3 = v7;
						tetrahedrons[tetrahedronCount].v4 = v3;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v2;
						tetrahedrons[tetrahedronCount].v2 = v5;
						tetrahedrons[tetrahedronCount].v3 = v4;
						tetrahedrons[tetrahedronCount].v4 = v1;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v2;
						tetrahedrons[tetrahedronCount].v2 = v7;
						tetrahedrons[tetrahedronCount].v3 = v4;
						tetrahedrons[tetrahedronCount].v4 = v5;
						++tetrahedronCount;
					}
					else
					{
						tetrahedrons[tetrahedronCount].v1 = v6;
						tetrahedrons[tetrahedronCount].v2 = v1;
						tetrahedrons[tetrahedronCount].v3 = v3;
						tetrahedrons[tetrahedronCount].v4 = v2;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v6;
						tetrahedrons[tetrahedronCount].v2 = v8;
						tetrahedrons[tetrahedronCount].v3 = v1;
						tetrahedrons[tetrahedronCount].v4 = v5;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v6;
						tetrahedrons[tetrahedronCount].v2 = v3;
						tetrahedrons[tetrahedronCount].v3 = v8;
						tetrahedrons[tetrahedronCount].v4 = v7;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v1;
						tetrahedrons[tetrahedronCount].v2 = v8;
						tetrahedrons[tetrahedronCount].v3 = v3;
						tetrahedrons[tetrahedronCount].v4 = v4;
						++tetrahedronCount;

						tetrahedrons[tetrahedronCount].v1 = v6;
						tetrahedrons[tetrahedronCount].v2 = v1;
						tetrahedrons[tetrahedronCount].v3 = v8;
						tetrahedrons[tetrahedronCount].v4 = v3;
						++tetrahedronCount;
					}
				}
			}
		}
		
		assert(tetrahedronCount == 5 * H * W * D);
	}

	int GetRowVertexCount() const
	{
		return H + 1;
	}

	int GetColumnVertexCount() const
	{
		return W + 1;
	}

	int GetDepthVertexCount() const
	{
		return D + 1;
	}
	
	int GetVertex(int i, int j, int k) const
	{
		assert(i < H + 1);
		assert(j < W + 1);
		assert(k < D + 1);
		return k + (D + 1) * (j + (W + 1) * i);
	}
};

#endif

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

#ifndef B3_DEBUGDRAW_H
#define B3_DEBUGDRAW_H

#include <bounce_softbody/common/math/mat33.h>
#include <bounce_softbody/common/math/transform.h>
#include <bounce_softbody/common/graphics/debug_points.h>
#include <bounce_softbody/common/graphics/debug_lines.h>
#include <bounce_softbody/common/graphics/debug_triangles.h>

// This contains the primitives to be used by the debug draw utilities.
// You must setup this structure with the pointers before calling any debug draw function.
// None of the pointers must be null pointers.
struct b3DebugDrawData
{
	b3DebugPoints* points = nullptr;
	b3DebugLines* lines = nullptr;
	b3DebugTriangles* triangles = nullptr;
};

// The debug draw utility section.

// Draw a point.
inline void b3DrawPoint(b3DebugDrawData* data, const b3Vec3& p, scalar size, const b3Color& color, bool depthEnabled = true)
{
	data->points->Draw(p, size, color, depthEnabled);
}

// Draw a segment.
inline void b3DrawSegment(b3DebugDrawData* data, const b3Vec3& p1, const b3Vec3& p2, const b3Color& color, bool depthEnabled = true)
{
	data->lines->Draw(p1, p2, color, depthEnabled);
}

// Draw a triangle.
inline void b3DrawTriangle(b3DebugDrawData* data, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled = true)
{
	data->lines->Draw(p1, p2, color, depthEnabled);
	data->lines->Draw(p2, p3, color, depthEnabled);
	data->lines->Draw(p3, p1, color, depthEnabled);
}

// Draw a solid triangle.
inline void b3DrawSolidTriangle(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled = true)
{
	data->triangles->Draw(normal, p1, p2, p3, color, depthEnabled);
}

static inline b3Vec3 b3MakeVec3(int vertexStride, const void* vertexBase, int vtx)
{
	scalar* p = (scalar*)((char*)vertexBase + vertexStride * vtx);
	return b3Vec3(p[0], p[1], p[2]);
}

// Draw a polygon.
inline void b3DrawPolygon(b3DebugDrawData* data, const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 p1 = b3MakeVec3(vertexStride, vertices, count - 1);
	for (uint32 i = 0; i < count; ++i)
	{
		b3Vec3 p2 = b3MakeVec3(vertexStride, vertices, i);

		data->lines->Draw(p1, p2, color, depthEnabled);

		p1 = p2;
	}
}

// Draw a solid polygon with vertices ordered CCW.
inline void b3DrawSolidPolygon(b3DebugDrawData* data, const b3Vec3& normal, const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 p1 = b3MakeVec3(vertexStride, vertices, 0);
	for (uint32 i = 1; i < count - 1; ++i)
	{
		b3Vec3 p2 = b3MakeVec3(vertexStride, vertices, i);
		b3Vec3 p3 = b3MakeVec3(vertexStride, vertices, i + 1);

		data->triangles->Draw(normal, p1, p2, p3, color, depthEnabled);
	}
}

// Draw a circle.
template <uint32 E = 20>
inline void b3DrawCircle(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 n1, n3;
	b3ComputeBasis(normal, n1, n3);
	
	scalar kAngleInc = scalar(2) * B3_PI / scalar(E);
	
	b3Quat q;
	q.SetAxisAngle(normal, kAngleInc);

	b3Vec3 p1 = center + radius * n1;
	for (uint32 i = 0; i < E; ++i)
	{
		b3Vec3 n2 = b3Mul(q, n1);
		b3Vec3 p2 = center + radius * n2;

		data->lines->Draw(p1, p2, color, depthEnabled);

		n1 = n2;
		p1 = p2;
	}
}

// Draw a solid circle.
template<uint32 E = 20>
void b3DrawSolidCircle(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 n1, n3;
	b3ComputeBasis(normal, n1, n3);

	scalar kAngleInc = scalar(2) * B3_PI / scalar(E);
	
	b3Quat q;
	q.SetAxisAngle(normal, kAngleInc);

	b3Vec3 p1 = center + radius * n1;
	for (uint32 i = 0; i < E; ++i)
	{
		b3Vec3 n2 = b3Mul(q, n1);
		b3Vec3 p2 = center + radius * n2;

		data->triangles->Draw(normal, center, p1, p2, color, depthEnabled);

		n1 = n2;
		p1 = p2;
	}
}

// A (H + 1) x (W + 1) UV sphere mesh stored in row-major order.
// v(i, j) = i * (W + 1) + j
template<uint32 H = 1, uint32 W = 1>
struct b3SphereMesh
{
	uint32 vertexCount;
	b3Vec3 vertices[(H + 1) * (W + 1)];
	uint32 indexCount;
	uint32 indices[3 * 2 * H * W];
	
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
				vertices[vertex] = p;
				++vertexCount;
			}
		}
		
		B3_ASSERT(vertexCount == (H + 1) * (W + 1));
		
		// Build triangles
		indexCount = 0;
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

				indices[indexCount++] = v1;
				indices[indexCount++] = v2;
				indices[indexCount++] = v3;

				indices[indexCount++] = v3;
				indices[indexCount++] = v4;
				indices[indexCount++] = v1;
			}
		}

		B3_ASSERT(indexCount == 3 * 2 * H * W);
	}

	uint32 GetVertex(uint32 i, uint32 j)
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};

// Draw a sphere.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawSphere(b3DebugDrawData* data, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3SphereMesh<H, W> sphere;
	
	for (uint32 i = 0; i < sphere.indexCount / 3; ++i)
	{
		uint32 v1 = sphere.indices[3 * i];
		uint32 v2 = sphere.indices[3 * i + 1];
		uint32 v3 = sphere.indices[3 * i + 2];

		b3Vec3 p1 = sphere.vertices[v1];
		p1 *= radius;
		p1 += center;

		b3Vec3 p2 = sphere.vertices[v2];
		p2 *= radius;
		p2 += center;

		b3Vec3 p3 = sphere.vertices[v3];
		p3 *= radius;
		p3 += center;
		
		b3DrawTriangle(data, p1, p2, p3, color, depthEnabled);
	}
}

// Draw a solid sphere.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawSolidSphere(b3DebugDrawData* data, const b3Quat& rotation, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3SphereMesh<H, W> sphere;
	
	b3Transform xf;
	xf.rotation = rotation;
	xf.translation = center;

	for (uint32 i = 0; i < sphere.indexCount / 3; ++i)
	{
		uint32 v1 = sphere.indices[3 * i];
		uint32 v2 = sphere.indices[3 * i + 1];
		uint32 v3 = sphere.indices[3 * i + 2];

		b3Vec3 p1 = sphere.vertices[v1];
		
		b3Vec3 n1 = p1;

		p1 *= radius;
		p1 = b3Mul(xf, p1);

		b3Vec3 p2 = sphere.vertices[v2];

		b3Vec3 n2 = p2;
		
		p2 *= radius;
		p2 = b3Mul(xf, p2);

		b3Vec3 p3 = sphere.vertices[v3];

		b3Vec3 n3 = p3;
		
		p3 *= radius;
		p3 = b3Mul(xf, p3);

		b3Vec3 n = (n1 + n2 + n3) / scalar(3);
		n = b3Mul(xf.rotation, n);

		data->triangles->Draw(n, p1, p2, p3, color, depthEnabled);
	}
}

// A cylinder mesh.
template<uint32 H = 1, uint32 W = 1>
struct b3CylinderMesh
{
	uint32 vertexCount;
	b3Vec3 vertices[(H + 1) * (W + 1)];
	uint32 indexCount;
	uint32 indices[3 * 2 * H * W + 3 * 2 * (((W + 1) - 1) - 1)];
	
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
				vertices[vertex] = p;
				++vertexCount;
			}
		}
		
		B3_ASSERT(vertexCount == (H + 1) * (W + 1));
		
		// Build triangles
		indexCount = 0;
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

				indices[indexCount++] = v1;
				indices[indexCount++] = v2;
				indices[indexCount++] = v3;

				indices[indexCount++] = v3;
				indices[indexCount++] = v4;
				indices[indexCount++] = v1;
			}
		}

		B3_ASSERT(indexCount == 3 * 2 * H * W);
		
		// Lower circle
		uint32 i1 = 0;
		for (uint32 i2 = i1 + 1; i2 < (W + 1) - 1; ++i2)
		{
			uint32 i3 = i2 + 1;
			
			uint32 v1 = GetVertex(0, i1);
			uint32 v2 = GetVertex(0, i2);
			uint32 v3 = GetVertex(0, i3);
			
			indices[indexCount++] = v1;
			indices[indexCount++] = v2;
			indices[indexCount++] = v3;
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
			indices[indexCount++] = v3;
			indices[indexCount++] = v2;
			indices[indexCount++] = v1;
		}
		
		B3_ASSERT(indexCount == 3 * 2 * H * W + 3 * 2 * (((W + 1) - 1) - 1));
	}
	
	uint32 GetVertex(uint32 i, uint32 j)
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};

// Draw a cylinder.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawCylinder(b3DebugDrawData* data, const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled = true)
{
	b3CylinderMesh<H, W> cylinder;
	
	b3Transform xf;
	xf.rotation = b3QuatRotationBetween(b3Vec3_y, axis);
	xf.translation = center;

	for (uint32 i = 0; i < cylinder.indexCount / 3; ++i)
	{
		uint32 v1 = cylinder.indices[3 * i];
		uint32 v2 = cylinder.indices[3 * i + 1];
		uint32 v3 = cylinder.indices[3 * i + 2];

		b3Vec3 p1 = cylinder.vertices[v1];
		
		p1.x *= radius;
		p1.y *= height;
		p1.z *= radius;
		p1 = b3Mul(xf, p1);
		
		b3Vec3 p2 = cylinder.vertices[v2];
		
		p2.x *= radius;
		p2.y *= height;
		p2.z *= radius;
		p2 = b3Mul(xf, p2);

		b3Vec3 p3 = cylinder.vertices[v3];
		
		p3.x *= radius;
		p3.y *= height;
		p3.z *= radius;
		p3 = b3Mul(xf, p3);

		b3DrawTriangle(data, p1, p2, p3, color, depthEnabled);
	}
}

// Draw a solid cylinder.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawSolidCylinder(b3DebugDrawData* data, const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled = true)
{
	b3CylinderMesh<H, W> cylinder;
	
	b3Transform xf;
	xf.rotation = b3QuatRotationBetween(b3Vec3_y, axis);
	xf.translation = center;

	for (uint32 i = 0; i < cylinder.indexCount / 3; ++i)
	{
		uint32 v1 = cylinder.indices[3 * i];
		uint32 v2 = cylinder.indices[3 * i + 1];
		uint32 v3 = cylinder.indices[3 * i + 2];

		b3Vec3 p1 = cylinder.vertices[v1];
		b3Vec3 p2 = cylinder.vertices[v2];
		b3Vec3 p3 = cylinder.vertices[v3];
		
		b3Vec3 n = b3Cross(p2 - p1, p3 - p1);
		n.Normalize();
		
		n = b3Mul(xf.rotation, n);
		
		p1.x *= radius;
		p1.y *= height;
		p1.z *= radius;
		p1 = b3Mul(xf, p1);

		p2.x *= radius;
		p2.y *= height;
		p2.z *= radius;
		p2 = b3Mul(xf, p2);
		
		p3.x *= radius;
		p3.y *= height;
		p3.z *= radius;
		p3 = b3Mul(xf, p3);

		data->triangles->Draw(n, p1, p2, p3, color, depthEnabled);
	}
}

// Draw a capsule.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawCapsule(b3DebugDrawData* data, const b3Vec3& c1, const b3Vec3& c2, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3DrawSphere<H, W>(data, c1, radius, color);
	if (b3LengthSquared(c1 - c2) > B3_EPSILON * B3_EPSILON)
	{
		{
			scalar height = b3Length(c1 - c2);
			b3Vec3 axis = (c1 - c2) / height;
			b3Vec3 center = scalar(0.5) * (c1 + c2);

			b3DrawCylinder<H, W>(data, axis, center, radius, height, color, depthEnabled);
		}

		b3DrawSphere<H, W>(data, c2, radius, color, depthEnabled);
	}
}

// Draw a capsule in solid rendering mode.
template<uint32 H = 20, uint32 W = 20>
inline void b3DrawSolidCapsule(b3DebugDrawData* data, const b3Quat& rotation, const b3Vec3& c1, const b3Vec3& c2, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3DrawSolidSphere<H, W>(data, rotation, c1, radius, color, depthEnabled);
	if (b3LengthSquared(c1 - c2) > B3_EPSILON * B3_EPSILON)
	{
		{
			scalar height = b3Length(c1 - c2);
			b3Vec3 axis = (c1 - c2) / height;
			b3Vec3 center = scalar(0.5) * (c1 + c2);
						
			b3DrawSolidCylinder<H, W>(data, axis, center, radius, height, color, depthEnabled);
		}

		b3DrawSolidSphere<H, W>(data, rotation, c2, radius, color, depthEnabled);
	}
}

// Draw a transform.
inline void b3DrawTransform(b3DebugDrawData* data, const b3Transform& xf, bool depthEnabled = true)
{
	scalar length = scalar(1);

	b3Vec3 translation = xf.translation;
	b3Quat rotation = xf.rotation;
	
	b3Vec3 A = translation + length * rotation.GetXAxis();
	b3Vec3 B = translation + length * rotation.GetYAxis();
	b3Vec3 C = translation + length * rotation.GetZAxis();

	data->lines->Draw(translation, A, b3Color_red, depthEnabled);
	data->lines->Draw(translation, B, b3Color_green, depthEnabled);
	data->lines->Draw(translation, C, b3Color_blue, depthEnabled);
}

// Draw an AABB.
inline void b3DrawAABB(b3DebugDrawData* data, const b3Vec3& lowerBound, const b3Vec3& upperBound, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 vs[8];

	vs[0] = lowerBound;
	vs[1] = b3Vec3(lowerBound.x, upperBound.y, lowerBound.z);
	vs[2] = b3Vec3(upperBound.x, upperBound.y, lowerBound.z);
	vs[3] = b3Vec3(upperBound.x, lowerBound.y, lowerBound.z);

	vs[4] = upperBound;
	vs[5] = b3Vec3(upperBound.x, lowerBound.y, upperBound.z);
	vs[6] = b3Vec3(lowerBound.x, lowerBound.y, upperBound.z);
	vs[7] = b3Vec3(lowerBound.x, upperBound.y, upperBound.z);

	data->lines->Draw(vs[0], vs[1], color, depthEnabled);
	data->lines->Draw(vs[1], vs[2], color, depthEnabled);
	data->lines->Draw(vs[2], vs[3], color, depthEnabled);
	data->lines->Draw(vs[3], vs[0], color, depthEnabled);

	data->lines->Draw(vs[4], vs[5], color, depthEnabled);
	data->lines->Draw(vs[5], vs[6], color, depthEnabled);
	data->lines->Draw(vs[6], vs[7], color, depthEnabled);
	data->lines->Draw(vs[7], vs[4], color, depthEnabled);

	data->lines->Draw(vs[2], vs[4], color, depthEnabled);
	data->lines->Draw(vs[5], vs[3], color, depthEnabled);

	data->lines->Draw(vs[6], vs[0], color, depthEnabled);
	data->lines->Draw(vs[1], vs[7], color, depthEnabled);
}

// Draw a grid.
template<uint32 H = 32, uint32 W = 32>
inline void b3DrawGrid(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& center, uint32 width, uint32 height, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 vs[(H + 1) * (W + 1)];
	
	uint32 h = b3Min(H + 1, height + 1);
	uint32 w = b3Min(W + 1, width + 1);
	
	b3Vec3 grid_center;
	grid_center.x = scalar(0.5) * scalar(width);
	grid_center.y = scalar(0);
	grid_center.z = scalar(0.5) * scalar(height);
	
	for (uint32 i = 0; i < h; ++i)
	{
		for (uint32 j = 0; j < w; ++j)
		{
			uint32 index = i * w + j;

			b3Vec3 v;
			v.x = scalar(j);
			v.y = scalar(0);
			v.z = scalar(i);

			v -= grid_center;
			
			vs[index] = v;
		}
	}

	b3Quat q = b3QuatRotationBetween(b3Vec3_y, normal);
	
	b3Color borderColor(scalar(0), scalar(0), scalar(0), scalar(1));
	b3Color centerColor(scalar(0.8), scalar(0.8), scalar(0.8), scalar(1));
	
	// Left to right lines
	for (uint32 i = 0; i < h; ++i)
	{
		uint32 iv1 = i * w + 0;
		uint32 iv2 = i * w + (w - 1);

		b3Vec3 v1 = b3Mul(q, vs[iv1]) + center;
		b3Vec3 v2 = b3Mul(q, vs[iv2]) + center;
		
		if (i == 0 || i == (h - 1))
		{
			data->lines->Draw(v1, v2, borderColor, depthEnabled);
			continue;
		}

		if (i == (h - 1) / 2)
		{
			data->lines->Draw(v1, v2, centerColor, depthEnabled);
			continue;
		}

		data->lines->Draw(v1, v2, color, depthEnabled);
	}

	// Up to bottom lines
	for (uint32 j = 0; j < w; ++j)
	{
		uint32 iv1 = 0 * w + j;
		uint32 iv2 = (h - 1) * w + j;

		b3Vec3 v1 = b3Mul(q, vs[iv1]) + center;
		b3Vec3 v2 = b3Mul(q, vs[iv2]) + center;
		
		if (j == 0 || j == (w - 1))
		{
			data->lines->Draw(v1, v2, borderColor, depthEnabled);
			continue;
		}

		if (j == (w - 1) / 2)
		{
			data->lines->Draw(v1, v2, centerColor, depthEnabled);
			continue;
		}

		data->lines->Draw(v1, v2, color, depthEnabled);
	}
}

// Draw a plane.
inline void b3DrawPlane(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 n1, n2;
	b3ComputeBasis(normal, n1, n2);

	scalar scale = scalar(2) * radius;

	b3Vec3 v1 = center - scale * (n1 - n2);
	b3Vec3 v2 = center + scale * (n1 - n2);
	b3Vec3 v3 = center + scale * (n1 + n2);
	b3Vec3 v4 = center - scale * (n1 + n2);

	data->lines->Draw(v1, v2, color, depthEnabled);
	data->lines->Draw(v2, v3, color, depthEnabled);
	data->lines->Draw(v3, v4, color, depthEnabled);
	data->lines->Draw(v4, v1, color, depthEnabled);
}

// Draw a solid plane.
inline void b3DrawSolidPlane(b3DebugDrawData* data, const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true)
{
	b3Vec3 n1, n2;
	b3ComputeBasis(normal, n1, n2);

	scalar scale = scalar(2) * radius;

	b3Vec3 v1 = center - scale * (n1 - n2);
	b3Vec3 v2 = center + scale * (n1 - n2);
	b3Vec3 v3 = center + scale * (n1 + n2);
	b3Vec3 v4 = center - scale * (n1 + n2);

	data->triangles->Draw(normal, v1, v2, v3, color, depthEnabled);
	data->triangles->Draw(normal, v3, v4, v1, color, depthEnabled);
}

#endif

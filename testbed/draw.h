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

#ifndef DRAW_H
#define DRAW_H

#include <bounce_softbody/common/draw.h>
#include <bounce_softbody/common/graphics/debugdraw.h>

struct b3DebugDrawData;

class Draw : public b3Draw
{
public:
	void DrawPoint(const b3Vec3& p, scalar size, const b3Color& color, bool depthEnabled)
	{
		b3DrawPoint(m_debugDrawData, p, size, color, depthEnabled);
	}
	
	void DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color, bool depthEnabled)
	{
		b3DrawSegment(m_debugDrawData, p1, p2, color, depthEnabled);
	}
	
	void DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled)
	{
		b3DrawTriangle(m_debugDrawData, p1, p2, p3, color, depthEnabled);
	}
	
	void DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidTriangle(m_debugDrawData, normal, p1, p2, p3, color);
	}
	
	void DrawPolygon(const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled)
	{
		b3DrawPolygon(m_debugDrawData, vertices, vertexStride, count, color, depthEnabled);	
	}
	
	void DrawSolidPolygon(const b3Vec3& normal, const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidPolygon(m_debugDrawData, normal, vertices, vertexStride, count, color, depthEnabled);
	}
	
	void DrawCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawCircle(m_debugDrawData, normal, center, radius, color, depthEnabled);
	}
	
	void DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidCircle(m_debugDrawData, normal, center, radius, color, depthEnabled);
	}
	
	void DrawPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawPlane(m_debugDrawData, normal, center, radius, color, depthEnabled);
	}
	
	void DrawSolidPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidPlane(m_debugDrawData, normal, center, radius, color, depthEnabled);
	}
	
	void DrawSphere(const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawSphere(m_debugDrawData, center, radius, color, depthEnabled);
	}

	void DrawSolidSphere(const b3Vec3& axis, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidSphere(m_debugDrawData, axis, center, radius, color, depthEnabled);
	}
	
	void DrawCylinder(const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled)
	{
		b3DrawCylinder(m_debugDrawData, axis, center, radius, height, color, depthEnabled);
	}
	
	void DrawSolidCylinder(const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidCylinder(m_debugDrawData, axis, center, radius, height, color, depthEnabled);
	}
	
	void DrawGrid(const b3Vec3& normal, const b3Vec3& center, uint32 width, uint32 height, const b3Color& color, bool depthEnabled)
	{
		b3DrawGrid(m_debugDrawData, normal, center, width, height, color, depthEnabled);
	}
	
	void DrawCapsule(const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawCapsule(m_debugDrawData, p1, p2, radius, color, depthEnabled);
	}
	
	void DrawSolidCapsule(const b3Vec3& axis, const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Color& color, bool depthEnabled)
	{
		b3DrawSolidCapsule(m_debugDrawData, axis, p1, p2, radius, color, depthEnabled);
	}
	
	void DrawAABB(const b3AABB& aabb, const b3Color& color, bool depthEnabled)
	{
		b3DrawAABB(m_debugDrawData, aabb.lowerBound, aabb.upperBound, color, depthEnabled);
	}
	
	void DrawTransform(const b3Transform& xf, bool depthEnabled)
	{
		b3DrawTransform(m_debugDrawData, xf, depthEnabled);
	}
	
	b3DebugDrawData* m_debugDrawData;
};

#endif
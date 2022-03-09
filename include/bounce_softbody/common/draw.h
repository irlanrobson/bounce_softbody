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

#ifndef B3_DRAW_H
#define B3_DRAW_H

#include <bounce_softbody/common/math/vec2.h>
#include <bounce_softbody/common/math/vec3.h>
#include <bounce_softbody/common/math/mat33.h>
#include <bounce_softbody/common/math/mat44.h>
#include <bounce_softbody/common/math/transform.h>
#include <bounce_softbody/common/graphics/color.h>
#include <bounce_softbody/collision/geometry/aabb.h>
#include <bounce_softbody/collision/geometry/plane.h>

// Debug draw interface.
class b3Draw
{
public:
	// Draw a point.
	virtual void DrawPoint(const b3Vec3& p, scalar size, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a line segment.
	virtual void DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a triangle with vertices ordered CCW.
	virtual void DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a solid triangle with vertices ordered CCW.
	virtual void DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a polygon with vertices ordered CCW.
	virtual void DrawPolygon(const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a solid polygon with vertices ordered CCW.
	virtual void DrawSolidPolygon(const b3Vec3& normal, const void* vertices, uint32 vertexStride, uint32 count, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a circle with center, normal, and radius.
	virtual void DrawCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a solid circle with center, normal, and radius.
	virtual void DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a plane with center, normal and radius.
	virtual void DrawPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a solid plane with center, normal and radius.
	virtual void DrawSolidPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a sphere with center, and radius.
	virtual void DrawSphere(const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a solid sphere with axis of rotation, center, and radius.
	virtual void DrawSolidSphere(const b3Vec3& axis, const b3Vec3& center, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a cylinder with axis, center, radius, and height.
	virtual void DrawCylinder(const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a solid cylinder with axis, center, radius, and height.
	virtual void DrawSolidCylinder(const b3Vec3& axis, const b3Vec3& center, scalar radius, scalar height, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a grid with orientation, center, width, and height.
	virtual void DrawGrid(const b3Vec3& normal, const b3Vec3& center, uint32 width, uint32 height, const b3Color& color, bool depthEnabled = true) = 0;
	
	// Draw a capsule with segment and radius.
	virtual void DrawCapsule(const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a solid capsule with axis, segment and radius.
	virtual void DrawSolidCapsule(const b3Vec3& axis, const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a AABB.
	virtual void DrawAABB(const b3AABB& aabb, const b3Color& color, bool depthEnabled = true) = 0;

	// Draw a transform.
	virtual void DrawTransform(const b3Transform& xf, bool depthEnabled = true) = 0;
};

#endif

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

#include <bounce_softbody/common/math/vec3.h>
#include <bounce_softbody/common/graphics/color.h>

#ifndef B3_DEBUG_POINTS_H
#define B3_DEBUG_POINTS_H

// Implement this interface to render the debug points.
class b3DebugPointsRenderer
{
public:
	virtual void AddPoint(const b3Vec3& position, const b3Color& color, scalar size) = 0;
	virtual void FlushPoints(bool depthEnabled) = 0;
};

struct b3DebugPoint
{
	b3Vec3 position;
	b3Color color;
	scalar size;
	bool depthEnabled;
};

// Draw points with batching and depth test controlling support.
class b3DebugPoints
{
public:
	b3DebugPoints(uint32 capacity)
	{
		m_capacity = capacity;
		m_count = 0;
		m_points = (b3DebugPoint*)b3Alloc(m_capacity * sizeof(b3DebugPoint));
		m_renderer = nullptr;
		m_drawEnabled = true;
	}

	~b3DebugPoints()
	{
		b3Free(m_points);
	}

	// Draw a point.
	void Draw(const b3Vec3& p, scalar size, const b3Color& color, bool depthEnabled = true)
	{
		if (m_drawEnabled == false)
		{
			return;
		}

		if (m_count == m_capacity)
		{
			Flush();
		}

		B3_ASSERT(m_count < m_capacity);

		b3DebugPoint point;
		point.position = p;
		point.size = size;
		point.color = color;
		point.depthEnabled = depthEnabled;

		m_points[m_count++] = point;
	}

	// Flush the points.
	void Flush()
	{
		if (m_renderer == nullptr)
		{
			m_count = 0;
			return;
		}

		// First pass: Depth test enabled.
		for (uint32 i = 0; i < m_count; ++i)
		{
			b3DebugPoint point = m_points[i];
			if (point.depthEnabled == true)
			{
				m_renderer->AddPoint(point.position, point.color, point.size);
			}
		}

		m_renderer->FlushPoints(true);

		// Second pass: Depth test disabled.
		for (uint32 i = 0; i < m_count; ++i)
		{
			b3DebugPoint point = m_points[i];
			if (point.depthEnabled == false)
			{
				m_renderer->AddPoint(point.position, point.color, point.size);
			}
		}

		m_renderer->FlushPoints(false);

		m_count = 0;
	}

	// Attach a renderer.
	void SetRenderer(b3DebugPointsRenderer* renderer) { m_renderer = renderer; }

	// Return the attached renderer.
	b3DebugPointsRenderer* GetRenderer() { return m_renderer; }
	const b3DebugPointsRenderer* GetRenderer() const { return m_renderer; }

	// Enable drawing the points.
	void EnableDraw(bool flag) { m_drawEnabled = flag; }

	// Is drawing enabled?
	bool IsDrawEnabled() const { return m_drawEnabled; }
private:
	uint32 m_capacity;
	uint32 m_count;
	b3DebugPoint* m_points;
	b3DebugPointsRenderer* m_renderer;
	bool m_drawEnabled;
};

#endif
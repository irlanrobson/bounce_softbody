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

#ifndef B3_DEBUG_LINES_H
#define B3_DEBUG_LINES_H

#include <bounce_softbody/common/math/vec3.h>
#include <bounce_softbody/common/graphics/color.h>

// Implement this interface to render the debug lines.
class b3DebugLinesRenderer
{
public:
	virtual void AddLine(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color) = 0;
	virtual void FlushLines(bool depthEnabled) = 0;
};

struct b3DebugLine
{
	b3Vec3 p1, p2;
	b3Color color;
	bool depthEnabled;
};

// Draw lines with batching and depth test controlling support.
class b3DebugLines
{
public:
	b3DebugLines(uint32 capacity)
	{
		m_capacity = capacity;
		m_count = 0;
		m_lines = (b3DebugLine*)b3Alloc(m_capacity * sizeof(b3DebugLine));
		m_renderer = nullptr;
		m_drawEnabled = true;
	}

	~b3DebugLines()
	{
		b3Free(m_lines);
	}

	// Draw a line.
	void Draw(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color, bool depthEnabled = true)
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

		b3DebugLine line;
		line.p1 = p1;
		line.p2 = p2;
		line.color = color;
		line.depthEnabled = depthEnabled;

		m_lines[m_count++] = line;
	}

	// Flush the lines.
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
			b3DebugLine line = m_lines[i];
			if (line.depthEnabled == true)
			{
				m_renderer->AddLine(line.p1, line.p2, line.color);
			}
		}

		m_renderer->FlushLines(true);

		// Second pass: Depth test disabled.
		for (uint32 i = 0; i < m_count; ++i)
		{
			b3DebugLine line = m_lines[i];
			if (line.depthEnabled == false)
			{
				m_renderer->AddLine(line.p1, line.p2, line.color);
			}
		}

		m_renderer->FlushLines(false);

		m_count = 0;
	}

	// Attach a renderer.
	void SetRenderer(b3DebugLinesRenderer* renderer) { m_renderer = renderer; }

	// Return the attached renderer.
	b3DebugLinesRenderer* GetRenderer() { return m_renderer; }
	const b3DebugLinesRenderer* GetRenderer() const { return m_renderer; }

	// Enable drawing the points.
	void EnableDraw(bool flag) { m_drawEnabled = flag; }

	// Is drawing enabled?
	bool IsDrawEnabled() const { return m_drawEnabled; }
private:
	uint32 m_capacity;
	uint32 m_count;
	b3DebugLine* m_lines;
	b3DebugLinesRenderer* m_renderer;
	bool m_drawEnabled;
};

#endif
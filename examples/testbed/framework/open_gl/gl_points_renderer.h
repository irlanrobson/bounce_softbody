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

#ifndef GL_POINTS_RENDERER_H
#define GL_POINTS_RENDERER_H

#include <bounce_softbody/common/graphics/debug_points.h>
#include "glad/glad.h"

class GLPointsRenderer : public b3DebugPointsRenderer
{
public:
	GLPointsRenderer(int point_capacity);
	~GLPointsRenderer();
	
	int GetVertexCapacity() { return m_vertex_capacity; }
	int GetVertexCount() { return m_vertex_count; }
	void SetMVP(float* mvp);
	
	void AddPoint(const b3Vec3& position, const b3Color& color, scalar size) override;
	void FlushPoints(bool depthEnabled) override;
private:
	void Vertex(float x, float y, float z, float r, float g, float b, float a, float point_size);
	void Flush();
	
	int m_vertex_capacity;
	float* m_positions;
	float* m_colors;
	float* m_sizes;
	int m_vertex_count;
	float m_mvp[16];

	GLuint m_vao;
	GLuint m_vbos[3];
	
	GLuint m_program;
	GLuint m_projection_uniform;
	GLuint m_position_attribute;
	GLuint m_color_attribute;
	GLuint m_size_attribute;
};

#endif
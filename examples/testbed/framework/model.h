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

#ifndef MODEL_H
#define MODEL_H

#include "open_gl/gl_points_renderer.h"
#include "open_gl/gl_lines_renderer.h"
#include "open_gl/gl_triangles_renderer.h"

#include <bounce_softbody/common/graphics/camera.h>
#include <bounce_softbody/common/graphics/debugdraw.h>

class Test;

extern b3Camera* g_camera;
extern b3DebugDrawData* g_debugDrawData;

class Model
{
public:
	Model();
	~Model();

	void Action_SetTest();
	void Action_ResetCamera();

	void Command_Press_Key(int button);
	void Command_Release_Key(int button);
	void Command_Press_Mouse_Left(const b3Vec2& ps);
	void Command_Release_Mouse_Left(const b3Vec2& ps);
	void Command_Move_Cursor(const b3Vec2& ps);

	void Command_ResizeCamera(scalar w, scalar h);
	void Command_RotateCameraX(scalar angle);
	void Command_RotateCameraY(scalar angle);
	void Command_TranslateCameraX(scalar d);
	void Command_TranslateCameraY(scalar d);
	void Command_ZoomCamera(scalar d);

	void Update();
private:
	b3Camera m_camera;
	b3DebugPoints m_points;
	b3DebugLines m_lines;
	b3DebugTriangles m_triangles;
	b3DebugDrawData m_debugDrawData;
	GLPointsRenderer m_pointsRenderer;
	GLLinesRenderer m_linesRenderer;
	GLTrianglesRenderer m_trianglesRenderer;
	Test* m_test;
	bool m_setTest;
};

inline void Model::Action_SetTest()
{
	m_setTest = true;
}

inline void Model::Action_ResetCamera()
{
	b3Vec3 position(20.0f, 20.0f, 40.0f);
	m_camera.SetPosition(position);
	m_camera.SetCenter(b3Vec3_zero);
}

inline void Model::Command_ResizeCamera(scalar w, scalar h)
{
	m_camera.SetWidth(w);
	m_camera.SetHeight(h);
}

inline void Model::Command_RotateCameraX(scalar angle)
{
	m_camera.AddPolarAngle(angle);
}

inline void Model::Command_RotateCameraY(scalar angle)
{
	m_camera.AddAzimuthalAngle(angle);
}

inline void Model::Command_TranslateCameraX(scalar d)
{
	m_camera.TranslateXAxis(d);
}

inline void Model::Command_TranslateCameraY(scalar d)
{
	m_camera.TranslateYAxis(d);
}

inline void Model::Command_ZoomCamera(scalar d)
{
	m_camera.AddRadius(d);
}

#endif

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


#include "model.h"
#include "view_model.h"
#include "test.h"

Model::Model() :
	m_settings(nullptr),
	m_testSettings(nullptr),
	m_points(512),
	m_lines(512),
	m_triangles(512),
	m_pointsRenderer(512),
	m_linesRenderer(512),
	m_trianglesRenderer(512)
{
	m_points.SetRenderer(&m_pointsRenderer);
	m_lines.SetRenderer(&m_linesRenderer);
	m_triangles.SetRenderer(&m_trianglesRenderer);

	m_debugDrawData.points = &m_points;
	m_debugDrawData.lines = &m_lines;
	m_debugDrawData.triangles = &m_triangles;
	
	m_test = nullptr;
	
	m_setTest = true;

	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

	Action_ResetCamera();
}

Model::~Model()
{
	delete m_test;
	m_test = nullptr;
}

void Model::Command_Press_Key(int button)
{
	m_test->KeyDown(button);
}

void Model::Command_Release_Key(int button)
{
	m_test->KeyUp(button);
}

static inline b3Ray ConvertScreenToWorldRay(const b3Camera& camera, const b3Vec2& ps)
{
	b3Vec3 origin = camera.BuildPosition();
	b3Vec3 direction = camera.ConvertScreenToWorld(b3Vec2(ps.x, ps.y));
	return b3Ray(origin, direction, camera.GetZFar());
}

void Model::Command_Press_Mouse_Left(const b3Vec2& ps)
{
	b3Ray rw = ConvertScreenToWorldRay(m_camera, ps);
	m_test->MouseLeftDown(rw);
}

void Model::Command_Release_Mouse_Left(const b3Vec2& ps)
{
	b3Ray rw = ConvertScreenToWorldRay(m_camera, ps);
	m_test->MouseLeftUp(rw);
}

void Model::Command_Move_Cursor(const b3Vec2& ps)
{
	b3Ray rw = ConvertScreenToWorldRay(m_camera, ps);
	m_test->MouseMove(rw);
}

void Model::Update()
{
	if (m_setTest)
	{
		Action_ResetCamera();
		
		delete m_test;
		
		TestArgs args;
		args.settings = m_settings;
		args.testSettings = m_testSettings;
		args.camera = &m_camera;
		args.debugDrawData = &m_debugDrawData;

		m_test = m_settings->tests[m_settings->testID].create(args);
		
		m_setTest = false;
		m_testSettings->pause = true;
	}

	if (m_testSettings->pause)
	{
		if (m_testSettings->singlePlay)
		{
			m_testSettings->inv_hertz = m_testSettings->hertz > 0.0f ? 1.0f / m_testSettings->hertz : 0.0f;
			m_testSettings->singlePlay = false;
		}
		else
		{
			m_testSettings->inv_hertz = 0.0f;
		}
	}
	else
	{
		m_testSettings->inv_hertz = m_testSettings->hertz > 0.0f ? 1.0f / m_testSettings->hertz : 0.0f;
	}
	
	m_points.EnableDraw(m_settings->drawPoints);
	m_lines.EnableDraw(m_settings->drawLines);
	m_triangles.EnableDraw(m_settings->drawTriangles);

	// Rendering code begins here.
	glViewport(0, 0, GLsizei(m_camera.GetWidth()), GLsizei(m_camera.GetHeight()));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	b3Mat44 V = m_camera.BuildViewMatrix();
	b3Mat44 P = m_camera.BuildProjectionMatrix();
	b3Mat44 VP = P * V;

	m_pointsRenderer.SetMVP(&VP.x.x);
	m_linesRenderer.SetMVP(&VP.x.x);
	m_trianglesRenderer.SetMVP(&VP.x.x);

	if (m_settings->drawGrid)
	{
		b3DrawGrid<20, 20>(&m_debugDrawData, b3Vec3_y, b3Vec3_zero, 20, 20, b3Color(0.4f, 0.4f, 0.4f, 1.0f));
	}

	m_test->Step();

	// Default order: Points over lines and lines over triangles.
	m_triangles.Flush();
	m_lines.Flush();
	m_points.Flush();
}

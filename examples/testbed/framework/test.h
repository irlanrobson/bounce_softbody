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

#ifndef TEST_H
#define TEST_H

#include "GLFW/glfw3.h"
#include "draw.h"
#include "view_model.h"

#include <bounce_softbody/bounce_softbody.h>
#include <bounce_softbody/common/graphics/camera.h>
#include <bounce_softbody/common/graphics/debugdraw.h>
#include <bounce_softbody/collision/geometry/ray.h>

extern void DrawString(const b3Camera* camera, const b3Color& color, const b3Vec2& ps, const char* string, ...);
extern void DrawString(const b3Camera* camera, const b3Color& color, const b3Vec3& pw, const char* string, ...);
extern void DrawString(const b3Color& color, const char* string, ...);

float RandomFloat(float a, float b);

struct TestArgs
{
	Settings* settings;
	TestSettings* testSettings;
	b3Camera* camera;
	b3DebugDrawData* debugDrawData;
};

class Test 
{
public:
	Test(const TestArgs& args)
	{
		m_settings = args.settings;
		m_testSettings = args.testSettings;
		m_camera = args.camera;
		m_debugDrawData = args.debugDrawData;
		m_draw.m_debugDrawData = m_debugDrawData;
		m_ray.origin.SetZero();
		m_ray.direction.Set(0.0f, 0.0f, -1.0f);
		m_ray.length = m_camera->GetZFar();
	}
	
	virtual ~Test() { }
	
	virtual void Step() { }
	
	virtual void MouseMove(const b3Ray& pw) { m_ray = pw; }
	virtual void MouseLeftDown(const b3Ray& pw) { }
	virtual void MouseLeftUp(const b3Ray& pw) { }
	virtual void KeyDown(int button) { }
	virtual void KeyUp(int button) { }

	virtual void BeginDragging() { }
	virtual void EndDragging() { }
protected:
	Settings* m_settings;
	TestSettings* m_testSettings;
	Draw m_draw;
	b3Ray m_ray;
	b3Camera* m_camera;
	b3DebugDrawData* m_debugDrawData;
};

#endif

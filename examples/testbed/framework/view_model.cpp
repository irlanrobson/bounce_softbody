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

#include "view_model.h"
#include "model.h"

#include "GLFW/glfw3.h"

#include "test.h"
#include "tests/body.h"
#include "tests/sphere_contact.h"
#include "tests/capsule_contact.h"
#include "tests/triangle_contact.h"
#include "tests/box_contact.h"
#include "tests/grid_contact.h"
#include "tests/sphere_mesh_sdf_contact.h"
#include "tests/cylinder_mesh_sdf_contact.h"
#include "tests/pinned_cloth.h"
#include "tests/particle_types.h"
#include "tests/stretch_mapping.h"
#include "tests/cloth_tearing.h"
#include "tests/cloth_element.h"
#include "tests/sheet.h"
#include "tests/node_types.h"

ViewModel::ViewModel(Model* model, GLFWwindow* window)
{
	m_model = model;
	m_model->SetSettings(&m_settings);
	m_model->SetTestSettings(&m_testSettings);

	m_window = window;
	m_ps0.SetZero();

	m_settings.RegisterTest("Sphere Contact", &SphereContact::Create);
	m_settings.RegisterTest("Capsule Contact", &CapsuleContact::Create);
	m_settings.RegisterTest("Triangle Contact", &TriangleContact::Create);
	m_settings.RegisterTest("Box Contact", &BoxContact::Create);
	m_settings.RegisterTest("Grid Contact", &GridContact::Create);
	m_settings.RegisterTest("Sphere Mesh SDF Contact", &SphereMeshSDFContact::Create);
	m_settings.RegisterTest("Cylinder Mesh SDF Contact", &CylinderMeshSDFContact::Create);
	m_settings.RegisterTest("Pinned Cloth", &PinnedCloth::Create );
	m_settings.RegisterTest("Particle Types", &ParticleTypes::Create);
	m_settings.RegisterTest("Stretch Mapping", &StretchMapping::Create);
	m_settings.RegisterTest("Cloth Tearing", &ClothTearing::Create);
	m_settings.RegisterTest("Cloth Element", &ClothElement::Create);
	m_settings.RegisterTest("Sheet", &Sheet::Create);
	m_settings.RegisterTest("Node Types", &NodeTypes::Create);
}

b3Vec2 ViewModel::GetCursorPosition() const
{
	double x, y;
	glfwGetCursorPos(m_window, &x, &y);
	return b3Vec2(scalar(x), scalar(y));
}

void ViewModel::Action_SetTest()
{
	m_model->Action_SetTest();
}

void ViewModel::Action_PreviousTest()
{
	m_settings.testID = b3Clamp(m_settings.testID - 1, 0, int(m_settings.testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_NextTest()
{
	m_settings.testID = b3Clamp(m_settings.testID + 1, 0, int(m_settings.testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_PlayPause()
{
	m_testSettings.pause = !m_testSettings.pause;
}

void ViewModel::Action_SinglePlay()
{
	m_testSettings.pause = true;
	m_testSettings.singlePlay = true;
}

void ViewModel::Action_ResetCamera()
{
	m_model->Action_ResetCamera();
}

void ViewModel::Event_SetWindowSize(int w, int h)
{
	m_model->Command_ResizeCamera(scalar(w), scalar(h));
}

void ViewModel::Event_Press_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		if (button == GLFW_KEY_DOWN)
		{
			m_model->Command_ZoomCamera(1.0f);
		}

		if (button == GLFW_KEY_UP)
		{
			m_model->Command_ZoomCamera(-1.0f);
		}
	}
	else
	{
		m_model->Command_Press_Key(button);
	}
}

void ViewModel::Event_Release_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (!shiftDown)
	{
		m_model->Command_Release_Key(button);
	}
}

void ViewModel::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Press_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Release_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Move_Cursor(float x, float y)
{
	b3Vec2 ps(x, y);
	
	b3Vec2 dp = ps - m_ps0;

	m_ps0 = ps;

	b3Vec2 n = b3Normalize(dp);

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	bool leftDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
	bool rightDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	if (shiftDown)
	{
		if (leftDown)
		{
			scalar ax = -0.005f * B3_PI * n.x;
			scalar ay = -0.005f * B3_PI * n.y;

			m_model->Command_RotateCameraY(ax);
			m_model->Command_RotateCameraX(ay);
		}

		if (rightDown)
		{
			scalar tx = 0.2f * n.x;
			scalar ty = -0.2f * n.y;

			m_model->Command_TranslateCameraX(tx);
			m_model->Command_TranslateCameraY(ty);
		}
	}
	else
	{
		m_model->Command_Move_Cursor(ps);
	}
}

void ViewModel::Event_Scroll(float dx, float dy)
{
	b3Vec2 n(dx, dy);
	n.Normalize();

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		m_model->Command_ZoomCamera(1.0f * n.y);
	}
}

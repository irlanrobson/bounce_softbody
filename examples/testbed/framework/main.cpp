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

#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "model.h"
#include "view_model.h"
#include "view.h"

#include <stdio.h>
#include <thread>
#include <chrono>

static GLFWwindow* s_window;

// MVVM pattern
// See https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93viewmodel
static Model* s_model;
static ViewModel* s_viewModel;
static View* s_view;

static void glfwErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error, description);
}

static void WindowSize(GLFWwindow* ww, int w, int h)
{
	s_view->Event_SetWindowSize(w, h);
}

static void CursorMove(GLFWwindow* w, double x, double y)
{
	s_view->Event_Move_Cursor(float(x), float(y));
}

static void WheelScroll(GLFWwindow* w, double dx, double dy)
{
	s_view->Event_Scroll(float(dx), float(dy));
}

static void MouseButton(GLFWwindow* w, int button, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
	{
		s_view->Event_Press_Mouse(button);
		break;
	}
	case GLFW_RELEASE:
	{
		s_view->Event_Release_Mouse(button);
		break;
	}
	default:
	{
		break;
	}
	}
}

static void KeyButton(GLFWwindow* w, int button, int scancode, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
	{
		s_view->Event_Press_Key(button);
		break;
	}
	case GLFW_RELEASE:
	{
		s_view->Event_Release_Key(button);
		break;
	}
	default:
	{
		break;
	}
	}
}

static void Run()
{
	int w, h;
	glfwGetWindowSize(s_window, &w, &h);
	s_view->Event_SetWindowSize(uint32(w), uint32(h));

	std::chrono::duration<double> sleepAdjust(0.0);

	while (glfwWindowShouldClose(s_window) == 0)
	{
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		
		s_view->BeginInterface();

		s_model->Update();

		s_view->Interface();

		s_view->RenderInterface();

		glfwSwapBuffers(s_window);
		glfwPollEvents();

		// Box2D.
		// Throttle to cap at 60Hz. This adaptive using a sleep adjustment. This could be improved by
		// using mm_pause or equivalent for the last millisecond.
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> target(1.0 / 60.0);
		std::chrono::duration<double> timeUsed = t2 - t1;
		std::chrono::duration<double> sleepTime = target - timeUsed + sleepAdjust;
		if (sleepTime > std::chrono::duration<double>(0))
		{
			std::this_thread::sleep_for(sleepTime);
		}

		std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
		std::chrono::duration<double> frameTime = t3 - t1;

		// Compute the sleep adjustment using a low pass filter
		sleepAdjust = 0.9 * sleepAdjust + 0.1 * (target - frameTime);
	}
}

int main(int argc, char** args)
{
#if defined(_WIN32)
	// Report memory leaks
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetBreakAlloc();
#endif

	glfwSetErrorCallback(glfwErrorCallback);

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

#if __APPLE__
	const char* glslVersion = "#version 150";
#else
	const char* glslVersion = NULL;
#endif

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create window
	char buffer[128];
	sprintf(buffer, "Bounce Testbed Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);

	bool fullscreen = false;
	if (fullscreen)
	{
		s_window = glfwCreateWindow(1920, 1080, buffer, glfwGetPrimaryMonitor(), NULL);
	}
	else
	{
		s_window = glfwCreateWindow(1280, 720, buffer, NULL, NULL);
	}

	if (s_window == NULL)
	{
		fprintf(stderr, "Failed to create GLFW window\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(s_window);

	// Load OpenGL functions using glad
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		fprintf(stderr, "Failed to load OpenGL functions using glad\n");
		glfwTerminate();
		return -1;
	}

	printf("GL %d.%d\n", GLVersion.major, GLVersion.minor);
	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetWindowSizeCallback(s_window, WindowSize);
	glfwSetCursorPosCallback(s_window, CursorMove);
	glfwSetScrollCallback(s_window, WheelScroll);
	glfwSetMouseButtonCallback(s_window, MouseButton);
	glfwSetKeyCallback(s_window, KeyButton);
	//glfwSwapInterval(1);

	s_model = new Model;
	s_viewModel = new ViewModel(s_model, s_window);
	s_view = new View(s_viewModel, s_window, glslVersion);

	Run();

	delete s_view;
	s_view = nullptr;

	delete s_viewModel;
	s_viewModel = nullptr;

	delete s_model;
	s_model = nullptr;

	glfwTerminate();
	s_window = nullptr;

	return 0;
}

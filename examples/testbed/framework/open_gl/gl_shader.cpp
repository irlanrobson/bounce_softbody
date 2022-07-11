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

#include "gl_shader.h"

#include <stdlib.h> // malloc, free, NULL
#include <stdio.h>
#include <assert.h>

void GLCheckGLError()
{
	GLenum error_code = glGetError();
	if (error_code != GL_NO_ERROR)
	{
		printf("OpenGL error code = %d\n", error_code);
		assert(false);
	}
}

void GLPrintLog(GLuint id)
{
	GLint log_length = 0;
	if (glIsShader(id))
	{
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &log_length);
	}
	else if (glIsProgram(id))
	{
		glGetProgramiv(id, GL_INFO_LOG_LENGTH, &log_length);
	}
	else
	{
		printf("Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(id))
	{
		glGetShaderInfoLog(id, log_length, NULL, log);
	}
	else if (glIsProgram(id))
	{
		glGetProgramInfoLog(id, log_length, NULL, log);
	}

	printf("%s", log);
	free(log);
}

static GLuint GLCreateShader(const char* source, GLenum type)
{
	GLuint shader = glCreateShader(type);

	const char* sources[] = { source };
	glShaderSource(shader, 1, sources, NULL);
	glCompileShader(shader);

	GLint status = GL_FALSE;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		printf("Error compiling %d shader.\n", type);
		GLPrintLog(shader);
		glDeleteShader(shader);
		assert(false);
		return 0;
	}

	return shader;
}

GLuint GLCreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vs_id = GLCreateShader(vs, GL_VERTEX_SHADER);
	GLuint fs_id = GLCreateShader(fs, GL_FRAGMENT_SHADER);
	assert(vs_id != 0 && fs_id != 0);

	GLuint program = glCreateProgram();
	glAttachShader(program, vs_id);
	glAttachShader(program, fs_id);
	glLinkProgram(program);

	glDeleteShader(vs_id);
	glDeleteShader(fs_id);

	GLint status = GL_FALSE;
	glGetProgramiv(program, GL_LINK_STATUS, &status);
	assert(status != GL_FALSE);

	return program;
}

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

#ifndef BODY_H
#define BODY_H

#include "test.h"
#include "uniform_body.h"
#include "body_dragger.h"

class Body : public Test
{
public:
	Body(const TestArgs& args) : Test(args)
	{
		m_body = nullptr;
		m_bodyDragger = nullptr;
	}

	~Body()
	{
		delete m_bodyDragger;
		delete m_body;
	}

	void Step()
	{
		Test::Step();

		m_body->Step(m_testSettings->inv_hertz,
			m_testSettings->forceIterations,
			m_testSettings->forceSubIterations);
		
		Draw();
		
		if (m_bodyDragger->IsDragging())
		{
			b3Vec3 pA = m_bodyDragger->GetPointA();
			b3Vec3 pB = m_bodyDragger->GetPointB();

			b3DrawPoint(m_debugDrawData, pA, 4.0f, b3Color_green);
			b3DrawPoint(m_debugDrawData, pB, 4.0f, b3Color_green);
			b3DrawSegment(m_debugDrawData, pA, pB, b3Color_white);
		}

		extern uint32 b3_forceSolverIterations;
		extern uint32 b3_forceSolverMinSubIterations;
		extern uint32 b3_forceSolverMaxSubIterations;

		DrawString(b3Color_white, "Iterations = %d", b3_forceSolverIterations);
		DrawString(b3Color_white, "Sub-iterations [min] [max] = [%d] [%d]", b3_forceSolverMinSubIterations, b3_forceSolverMaxSubIterations);

		scalar E = m_body->GetEnergy();
		DrawString(b3Color_white, "E = %f", E);
	}
	
	virtual void Draw()
	{
		m_body->DebugDraw(&m_draw);
	}
	
	void MouseMove(const b3Ray& pw)
	{
		Test::MouseMove(pw);

		if (m_bodyDragger->IsDragging())
		{
			m_bodyDragger->Drag();
		}
	}

	void MouseLeftDown(const b3Ray& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_bodyDragger->IsDragging() == false)
		{
			m_bodyDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_bodyDragger->IsDragging() == true)
		{
			m_bodyDragger->StopDragging();
		}
	}

	UniformBody* m_body;
	BodyDragger* m_bodyDragger;
};

#endif

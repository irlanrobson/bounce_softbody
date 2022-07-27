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

#ifndef PARTICLE_TYPES_H
#define PARTICLE_TYPES_H

class ParticleTypes : public PinnedCloth
{
public:
	ParticleTypes(const TestArgs& args) : PinnedCloth(args) { }

	void Step()
	{
		PinnedCloth::Step();

		DrawString(b3Color_white, "S - Static");
		DrawString(b3Color_white, "D - Dynamic");
		DrawString(b3Color_white, "K - Kinematic");
		DrawString(b3Color_white, "Arrows - Apply Force/Velocity/Position");
	}

	void SetClothType(b3ParticleType type)
	{
		for (int i = 0; i < m_clothMesh.GetRowVertexCount(); ++i)
		{
			int v1 = m_clothMesh.GetVertex(i, 0);
			int v2 = m_clothMesh.GetVertex(i, m_clothMesh.GetColumnVertexCount() - 1);

			m_body->GetParticle(v1)->SetType(type);
			m_body->GetParticle(v2)->SetType(type);
		}

		for (int j = 0; j < m_clothMesh.GetColumnVertexCount(); ++j)
		{
			int v1 = m_clothMesh.GetVertex(0, j);
			int v2 = m_clothMesh.GetVertex(m_clothMesh.GetRowVertexCount() - 1, j);

			m_body->GetParticle(v1)->SetType(type);
			m_body->GetParticle(v2)->SetType(type);
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetClothType(e_staticParticle);
		}

		if (button == GLFW_KEY_K)
		{
			SetClothType(e_kinematicParticle);
		}

		if (button == GLFW_KEY_D)
		{
			SetClothType(e_dynamicParticle);
		}

		for (b3Particle* p = m_body->GetParticleList(); p; p = p->GetNext())
		{
			b3Vec3 d;
			d.SetZero();

			if (button == GLFW_KEY_LEFT)
			{
				d.x = -1.0f;
			}

			if (button == GLFW_KEY_RIGHT)
			{
				d.x = 1.0f;
			}

			if (button == GLFW_KEY_UP)
			{
				d.y = 1.0f;
			}

			if (button == GLFW_KEY_DOWN)
			{
				d.y = -1.0f;
			}

			if (button == GLFW_KEY_LEFT ||
				button == GLFW_KEY_RIGHT ||
				button == GLFW_KEY_UP ||
				button == GLFW_KEY_DOWN)
			{
				if (p->GetType() == e_staticParticle)
				{
					p->ApplyTranslation(d);
				}

				if (p->GetType() == e_kinematicParticle)
				{
					b3Vec3 v = p->GetVelocity();

					v += 5.0f * d;

					p->SetVelocity(v);
				}

				if (p->GetType() == e_dynamicParticle)
				{
					b3Vec3 f = 100.0f * d;

					p->ApplyForce(f);
				}
			}
		}
	}

	static Test* Create(const TestArgs& args)
	{
		return new ParticleTypes(args);
	}
};

#endif

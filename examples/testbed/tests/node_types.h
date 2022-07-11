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

#ifndef NODE_TYPES_H
#define NODE_TYPES_H

class NodeTypes : public Body
{
public:
	NodeTypes()
	{
		// Create soft body
		TetDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.elementYoungModulus = 1000.0f;
		def.elementPoissonRatio = 0.33f;
		def.thickness = 0.2f;
		def.friction = 0.6f;

		m_body = new UniformBody(def);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		for (int i = 0; i < m_mesh.GetRowVertexCount(); ++i)
		{
			for (int k = 0; k < m_mesh.GetDepthVertexCount(); ++k)
			{
				int v = m_mesh.GetVertex(i, 0, k);

				b3Particle* p = m_body->GetParticle(v);
				p->SetType(e_staticParticle);
			}
		}

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	void Step()
	{
		Body::Step();

		DrawString(b3Color_white, "S - Static");
		DrawString(b3Color_white, "D - Dynamic");
		DrawString(b3Color_white, "K - Kinematic");
		DrawString(b3Color_white, "Arrows - Apply Force/Velocity/Position");
	}

	void SetBodyType(b3ParticleType type)
	{
		for (int i = 0; i < m_mesh.GetRowVertexCount(); ++i)
		{
			for (int k = 0; k < m_mesh.GetDepthVertexCount(); ++k)
			{
				int v = m_mesh.GetVertex(i, 0, k);

				b3Particle* p = m_body->GetParticle(v);
				p->SetType(type);
			}
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetBodyType(e_staticParticle);
		}

		if (button == GLFW_KEY_K)
		{
			SetBodyType(e_kinematicParticle);
		}

		if (button == GLFW_KEY_D)
		{
			SetBodyType(e_dynamicParticle);
		}

		for (int i = 0; i < m_mesh.vertexCount; ++i)
		{
			b3Particle* p = m_body->GetParticle(i);

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

	static Test* Create()
	{
		return new NodeTypes;
	}

	GridTetMesh<2, 5, 2> m_mesh;
};

#endif

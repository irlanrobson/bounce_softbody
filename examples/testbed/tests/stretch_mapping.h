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

#ifndef STRETCH_MAPPING_H
#define STRETCH_MAPPING_H

// Hot/Cold color map
// See http://paulbourke.net/miscellaneous/colourspace/
static inline b3Color Color(scalar x, scalar a, scalar b)
{
	x = b3Clamp(x, a, b);

	scalar d = b - a;

	b3Color c(1.0f, 1.0f, 1.0f); 

	if (x < a + 0.25f * d) 
	{
		c.r = 0.0f;
		c.g = 4.0f * (x - a) / d;
		return c;
	}
	
	if (x < a + 0.5f * d) 
	{
		c.r = 0.0f;
		c.b = 1.0f + 4.0f * (a + 0.25f * d - x) / d;
		return c;
	}

	if (x < a + 0.75f * d) 
	{
		c.r = 4.0f * (x - a - 0.5f * d) / d;
		c.b = 0.0f;
		return c;
	}

	c.g = 1.0f + 4.0f * (a + 0.75f * d - x) / d;
	c.b = 0.0f;
	return c;
}

class StretchMapping : public Body
{
public:
	StretchMapping(const TestArgs& args) : Body(args)
	{
		// Create cloth
		ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.stretchingStiffness = 10000.0f;
		//def.stretchStiffnessDamping = 100.0f;

		m_body = new UniformBody(def);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		// Freeze some particles
		for (int j = 0; j < m_clothMesh.GetColumnVertexCount(); ++j)
		{
			int v = m_clothMesh.GetVertex(0, j);

			b3Particle* p = m_body->GetParticle(v);
			p->SetType(e_staticParticle);
		}

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	void Draw() override
	{
		b3Vec3* tension = (b3Vec3*) malloc(m_clothMesh.vertexCount * sizeof(b3Vec3));
		for (int i = 0; i < m_clothMesh.vertexCount; ++i)
		{
			tension[i].SetZero();
		}

		for (b3Force* f = m_body->GetForceList(); f; f = f->GetNext())
		{
			if (f->GetType() == e_stretchForce)
			{
				b3StretchForce* s = (b3StretchForce*)f;

				b3Vec3 f1 = s->GetActionForce1();
				b3Vec3 f2 = s->GetActionForce2();
				b3Vec3 f3 = s->GetActionForce3();

				b3Particle* p1 = s->GetParticle1();
				b3Particle* p2 = s->GetParticle2();
				b3Particle* p3 = s->GetParticle3();

				uint32 v1 = p1->GetUserIndex();
				uint32 v2 = p2->GetUserIndex();
				uint32 v3 = p3->GetUserIndex();

				tension[v1] += f1;
				tension[v2] += f2;
				tension[v3] += f3;
			}
		}
		
		for (b3Particle* p = m_body->GetParticleList(); p; p = p->GetNext())
		{
			if (p->GetType() == e_staticParticle)
			{
				b3DrawPoint(m_debugDrawData, p->GetPosition(), 4.0f, b3Color_white);
			}

			if (p->GetType() == e_kinematicParticle)
			{
				b3DrawPoint(m_debugDrawData, p->GetPosition(), 4.0f, b3Color_blue);
			}

			if (p->GetType() == e_dynamicParticle)
			{
				b3DrawPoint(m_debugDrawData, p->GetPosition(), 4.0f, b3Color_green);
			}
		}

		for (int i = 0; i < m_clothMesh.triangleCount; ++i)
		{
			Triangle triangle = m_clothMesh.GetTriangle(i);

			int vi1 = triangle.v1;
			int vi2 = triangle.v2;
			int vi3 = triangle.v3;

			b3Vec3 v1 = m_body->GetParticle(vi1)->GetPosition();
			b3Vec3 v2 = m_body->GetParticle(vi2)->GetPosition();
			b3Vec3 v3 = m_body->GetParticle(vi3)->GetPosition();

			b3DrawTriangle(m_debugDrawData, v1, v2, v3, b3Color_black);

			b3Vec3 c = (v1 + v2 + v3) / 3.0f;

			scalar s = 0.9f;

			v1 = s * (v1 - c) + c;
			v2 = s * (v2 - c) + c;
			v3 = s * (v3 - c) + c;

			b3Vec3 f1 = tension[vi1];
			scalar L1 = b3Length(f1);

			b3Vec3 f2 = tension[vi2];
			scalar L2 = b3Length(f2);

			b3Vec3 f3 = tension[vi3];
			scalar L3 = b3Length(f3);

			scalar L = (L1 + L2 + L3) / 3.0f;

			const scalar kMaxT = 30000.0f;
			b3Color color = Color(L, 0.0f, kMaxT);
			
			b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
			n1.Normalize();

			scalar r = 0.05f;

			{
				b3Vec3 x1 = v1 + r * n1;
				b3Vec3 x2 = v2 + r * n1;
				b3Vec3 x3 = v3 + r * n1;

				b3DrawSolidTriangle(m_debugDrawData, n1, x1, x2, x3, color);
			}

			{
				b3Vec3 n2 = -n1;

				b3Vec3 x1 = v1 + r * n2;
				b3Vec3 x2 = v2 + r * n2;
				b3Vec3 x3 = v3 + r * n2;

				b3DrawSolidTriangle(m_debugDrawData, n2, x3, x2, x1, color);
			}
		}

		free(tension);
	}

	static Test* Create(const TestArgs& args)
	{
		return new StretchMapping(args);
	}

	GridClothMesh<10, 10> m_clothMesh;
};

#endif

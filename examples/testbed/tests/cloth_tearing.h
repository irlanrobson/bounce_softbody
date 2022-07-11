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

#ifndef CLOTH_TEARING_H
#define CLOTH_TEARING_H

#include <bounce_softbody/common/template/array.h>
#include <bounce_softbody/collision/geometry/plane.h>

class ClothTearing : public Body
{
public:
	ClothTearing()	
	{
		m_body = new UniformBody();

		GridClothMesh<10, 10> m;

		b3Particle** particles = (b3Particle * *)malloc(m.vertexCount * sizeof(b3Particle*));
		for (int i = 0; i < m.vertexCount; ++i)
		{
			b3ParticleDef pd;
			pd.type = e_dynamicParticle;
			pd.position = m.GetVertexPosition(i);

			b3Particle* p = m_body->CreateParticle(pd);
			particles[i] = p;
		}

		for (int i = 0; i < m.triangleCount; ++i)
		{
			BodyMeshTriangle triangle = m.GetTriangle(i);
			
			int v1 = triangle.v1;
			int v2 = triangle.v2;
			int v3 = triangle.v3;

			b3Particle* p1 = particles[v1];
			b3Particle* p2 = particles[v2];
			b3Particle* p3 = particles[v3];

			b3TriangleFixtureDef tfd;
			tfd.p1 = p1;
			tfd.p2 = p2;
			tfd.p3 = p3;
			tfd.v1 = p1->GetPosition();
			tfd.v2 = p2->GetPosition();
			tfd.v3 = p3->GetPosition();
			tfd.density = 0.1f;

			m_body->CreateTriangle(tfd);

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p1, p2, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p2, p3, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p3, p1, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}
		}

		for (int i = 0; i < m.GetColumnVertexCount(); ++i)
		{
			int vertex = m.GetVertex(0, i);
			particles[vertex]->SetType(e_staticParticle);
		}

		free(particles);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
		m_bodyDragger->SetStaticDrag(false);
	}

	b3SpringForce* FindSpringForce(b3Particle* p1, b3Particle* p2)
	{
		for (b3Force* f = m_body->GetForceList(); f; f = f->GetNext())
		{
			if (f->GetType() != e_springForce)
			{
				continue;
			}

			b3SpringForce* sf = (b3SpringForce*)f;

			b3Particle* sp1 = sf->GetParticle1();
			b3Particle* sp2 = sf->GetParticle2();

			if (sp1 == p1 && sp2 == p2)
			{
				return sf;
			}

			if (sp1 == p2 && sp2 == p1)
			{
				return sf;
			}
		}

		return nullptr;
	}

	b3SpringForce* CreateSpringForce(const b3SpringForceDef& def)
	{
		b3SpringForce* sf = FindSpringForce(def.p1, def.p2);
		if (sf != nullptr)
		{
			return sf;
		}

		return (b3SpringForce*)m_body->CreateForce(def);
	}

	void DrawSpringForces()
	{
		for (b3Force* f = m_body->GetForceList(); f; f = f->GetNext())
		{
			if (f->GetType() != e_springForce)
			{
				continue;
			}

			b3SpringForce* s = (b3SpringForce*)f;

			b3Particle* p1 = s->GetParticle1();
			b3Particle* p2 = s->GetParticle2();

			b3DrawSegment(g_debugDrawData, p1->GetPosition(), p2->GetPosition(), b3Color_black);
		}
	}

	void Partition(b3Particle* p, const b3Plane& plane,
		b3Array<b3TriangleFixture*>& above,
		b3Array<b3TriangleFixture*>& below)
	{
		for (b3TriangleFixture* t = m_body->GetTriangleList(); t; t = t->GetNext())
		{
			b3Particle* p1 = t->GetParticle1();
			b3Particle* p2 = t->GetParticle2();
			b3Particle* p3 = t->GetParticle3();

			if (p1 != p && p2 != p && p3 != p)
			{
				continue;
			}

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();
			b3Vec3 x3 = p3->GetPosition();

			b3Vec3 center = (x1 + x2 + x3) / 3.0f;

			scalar distance = b3Distance(center, plane);
			if (distance > 0.0f)
			{
				above.PushBack(t);
			}
			else
			{
				below.PushBack(t);
			}
		}
	}

	bool HasSpring(const b3Array<b3TriangleFixture*>& triangles,
		b3Particle* pSplit, b3Particle* pOther)
	{
		for (uint32 i = 0; i < triangles.Count(); ++i)
		{
			b3TriangleFixture* triangle = triangles[i];

			b3Particle* tp1 = triangle->GetParticle1();
			b3Particle* tp2 = triangle->GetParticle2();
			b3Particle* tp3 = triangle->GetParticle3();

			// 1, 2
			if (tp1 == pSplit && tp2 == pOther)
			{
				return true;
			}

			// 2, 1
			if (tp2 == pSplit && tp1 == pOther)
			{
				return true;
			}

			// 2, 3
			if (tp2 == pSplit && tp3 == pOther)
			{
				return true;
			}

			// 3, 2
			if (tp3 == pSplit && tp2 == pOther)
			{
				return true;
			}

			// 3, 1
			if (tp3 == pSplit && tp1 == pOther)
			{
				return true;
			}

			// 1, 3
			if (tp1 == pSplit && tp3 == pOther)
			{
				return true;
			}
		}

		return false;
	}

	bool SplitParticle(b3Particle* pSplit, const b3Plane& plane)
	{
		// Collect triangles.
		b3StackArray<b3TriangleFixture*, 32> trianglesAbove, trianglesBelow;
		Partition(pSplit, plane, trianglesAbove, trianglesBelow);

		// There must be at least one triangle on each side of the plane.
		if (trianglesAbove.Count() == 0 || trianglesBelow.Count() == 0)
		{
			return false;
		}

		b3ParticleDef pdNew;
		pdNew.type = pSplit->GetType();
		pdNew.position = pSplit->GetPosition() - 0.2f * plane.normal;

		b3Particle* pNew = m_body->CreateParticle(pdNew);

		for (uint32 i = 0; i < trianglesBelow.Count(); ++i)
		{
			b3TriangleFixture* triangle = trianglesBelow[i];

			b3Particle* p1 = triangle->GetParticle1();
			b3Particle* p2 = triangle->GetParticle2();
			b3Particle* p3 = triangle->GetParticle3();

			m_body->DestroyTriangle(triangle);

			if (p1 == pSplit)
			{
				b3TriangleFixtureDef tdNew;
				tdNew.p1 = pNew;
				tdNew.p2 = p2;
				tdNew.p3 = p3;
				tdNew.v1 = pNew->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_body->CreateTriangle(tdNew);

				b3SpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p2;
					sNew.restLength = sf1->GetRestLength();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();

					m_body->CreateForce(sNew);

					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p3;
					sNew.p2 = pNew;
					sNew.restLength = sf2->GetRestLength();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
			
			if (p2 == pSplit)
			{
				b3TriangleFixtureDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = pNew;
				tdNew.p3 = p3;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = pNew->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_body->CreateTriangle(tdNew);

				b3SpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p1;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLength();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();

					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p2, p3);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p3;
					sNew.restLength = sf2->GetRestLength();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
			
			if (p3 == pSplit)
			{
				b3TriangleFixtureDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = p2;
				tdNew.p3 = pNew;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = pNew->GetPosition();

				m_body->CreateTriangle(tdNew);
				
				b3SpringForce* sf1 = FindSpringForce(p2, p3);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p2;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLength();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p1;
					sNew.restLength = sf2->GetRestLength();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
		}

		return true;
	}

	bool Tear()
	{
		b3Force* f = m_body->GetForceList();
		while (f)
		{
			if (f->GetType() != e_springForce)
			{
				f = f->GetNext();
				continue;
			}

			b3SpringForce* s = (b3SpringForce*)f;
			f = f->GetNext();

			b3Vec3 tension = s->GetActionForce();

			const scalar kMaxTension = 2000.0f;

			if (b3LengthSquared(tension) <= kMaxTension * kMaxTension)
			{
				continue;
			}

			b3Particle* p1 = s->GetParticle1();
			b3Particle* p2 = s->GetParticle2();

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();

			if (p1->GetType() == e_dynamicParticle)
			{
				b3Vec3 n = b3Normalize(x2 - x1);
				b3Plane plane(n, x1);

				bool wasSplit = SplitParticle(p1, plane);
				if (wasSplit)
				{
					return true;
				}
			}

			if (p2->GetType() == e_dynamicParticle)
			{
				b3Vec3 n = b3Normalize(x1 - x2);
				b3Plane plane(n, x2);

				bool wasSplit = SplitParticle(p2, plane);
				if (wasSplit)
				{
					return true;
				}
			}
		}

		return false;
	}

	void Step()
	{
		Body::Step();

		while (Tear());

		DrawSpringForces();
	}

	static Test* Create()
	{
		return new ClothTearing;
	}
};

#endif

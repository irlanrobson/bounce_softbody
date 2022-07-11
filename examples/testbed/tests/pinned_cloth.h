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

#ifndef PINNED_CLOTH_H
#define PINNED_CLOTH_H

class PinnedCloth : public Body
{
public:
	PinnedCloth()
	{
		// Create cloth
		ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.stretchingStiffness = 100000.0f;

		m_body = new UniformBody(def);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		// Freeze some particles
		for (int i = 0; i < m_clothMesh.GetRowVertexCount(); ++i)
		{
			int v1 = m_clothMesh.GetVertex(i, 0);
			int v2 = m_clothMesh.GetVertex(i, m_clothMesh.GetColumnVertexCount() - 1);

			m_body->GetParticle(v1)->SetType(e_staticParticle);
			m_body->GetParticle(v2)->SetType(e_staticParticle);
		}
		
		for (int j = 0; j < m_clothMesh.GetColumnVertexCount(); ++j)
		{
			int v1 = m_clothMesh.GetVertex(0, j);
			int v2 = m_clothMesh.GetVertex(m_clothMesh.GetRowVertexCount() - 1, j);

			m_body->GetParticle(v1)->SetType(e_staticParticle);
			m_body->GetParticle(v2)->SetType(e_staticParticle);
		}

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new PinnedCloth;
	}

	GridClothMesh<10, 10> m_clothMesh;
};

#endif

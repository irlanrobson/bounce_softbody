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

#ifndef SHEET_H
#define SHEET_H

class Sheet : public Body
{
public:
	Sheet()
	{
		// Downscale the block along the y axis
		b3Vec3 scale(1.0f, 0.5f, 1.0f);
		m_mesh.Scale(scale);

		// Create soft body
		TetDef def;
		def.mesh = &m_mesh;
		def.density = 0.3f;
		def.elementYoungModulus = 200.0f;

		m_body = new UniformBody(def);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		for (int j = 0; j < m_mesh.GetColumnVertexCount(); ++j)
		{
			int v = m_mesh.GetVertex(0, j, 0);

			b3Particle* p = m_body->GetParticle(v);
			p->SetType(e_staticParticle);
		}

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new Sheet;
	}

	GridTetMesh<1, 10, 10> m_mesh;
};

#endif

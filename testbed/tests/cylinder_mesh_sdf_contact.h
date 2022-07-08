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

#ifndef CYLINDER_MESH_SDF_CONTACT_H
#define CYLINDER_MESH_SDF_CONTACT_H

class CylinderMeshSDFContact : public Body
{
public:
	CylinderMeshSDFContact()
	{
		m_cylinderMesh.Scale(b3Vec3(3.0f, 3.0f, 3.0f));

		m_sdf.Build(&m_cylinderMesh, b3Vec3(1.0f, 1.0f, 1.0f), 1.0f);

		m_clothMesh.Translate(b3Vec3(0.0f, 10.0f, 0.0f));

		ClothDef def;
		def.mesh = &m_clothMesh;
		def.thickness = 0.2f;
		def.friction = 0.5f;
		m_body = new UniformBody(def);

		b3SDFShape sdfShape;
		sdfShape.m_sdf = &m_sdf;
		sdfShape.m_radius = 0.2f;
		sdfShape.m_xf.rotation = b3QuatRotationY(0.25f * B3_PI);

		b3WorldFixtureDef fixtureDef;
		fixtureDef.shape = &sdfShape;
		fixtureDef.friction = 0.5f;

		m_body->CreateFixture(fixtureDef);

		m_body->SetGravity(b3Vec3(0.0f, -10.0f, 0.0f));

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new CylinderMeshSDFContact;
	}

	GridClothMesh<10, 10> m_clothMesh;
	b3CylinderMesh<10, 10> m_cylinderMesh;
	b3SDF m_sdf;
};

#endif

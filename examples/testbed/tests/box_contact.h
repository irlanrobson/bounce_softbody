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

#ifndef BOX_CONTACT_H
#define BOX_CONTACT_H

class BoxContact : public Body
{
public:
	BoxContact()
	{
		m_mesh.Translate(b3Vec3(0.0f, 10.0f, 0.0f));
		
		ClothDef def;
		def.mesh = &m_mesh;
		def.thickness = 0.2f;
		def.friction = 0.8f;
		m_body = new UniformBody(def);
		
		b3BoxShape boxShape;
		boxShape.m_extents.Set(3.0f, 3.0f, 3.0f);
		boxShape.m_radius = 0.2f;

		b3WorldFixtureDef fixtureDef;
		fixtureDef.shape = &boxShape;
		fixtureDef.friction = 0.5f;
		
		m_body->CreateFixture(fixtureDef);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new BoxContact;
	}

	GridClothMesh<10, 10> m_mesh;
};

#endif

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

#ifndef CAPSULE_CONTACT_H
#define CAPSULE_CONTACT_H

class CapsuleContact : public Body
{
public:
	CapsuleContact()
	{
		m_mesh.Translate(b3Vec3(0.0f, 10.0f, 0.0f));

		ClothDef def;
		def.mesh = &m_mesh;
		def.radius = 0.2f;
		def.friction = 0.4f;
		m_body = new UniformBody(def);

		b3CapsuleShape capsuleShape;
		capsuleShape.m_center1.Set(0.0f, 0.0f, 5.0f);
		capsuleShape.m_center2.Set(0.0f, 0.0f, -5.0f);
		capsuleShape.m_radius = 2.0f;

		b3WorldFixtureDef fixtureDef;
		fixtureDef.shape = &capsuleShape;
		fixtureDef.friction = 0.5f;

		m_body->CreateFixture(fixtureDef);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_bodyDragger = new BodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new CapsuleContact;
	}

	GridClothMesh<10, 10> m_mesh;
};

#endif

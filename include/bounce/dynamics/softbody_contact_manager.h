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

#ifndef B3_SOFTBODY_CONTACT_MANAGER_H
#define B3_SOFTBODY_CONTACT_MANAGER_H

#include <bounce/dynamics/contacts/softbody_sphere_shape_contact.h>
#include <bounce/collision/broad_phase.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/list.h>

class b3SoftBody;

// Contact delegator for b3SoftBody.
class b3SoftBodyContactManager
{
public:
	b3SoftBodyContactManager();

	void AddContact(b3SoftBodySphereShape* s1, b3SoftBodyWorldShape* s2);
	void FindNewContacts();
	void UpdateContacts();

	b3SoftBodySphereAndShapeContact* CreateSphereAndShapeContact();
	void Destroy(b3SoftBodySphereAndShapeContact* c);

	b3BlockPool m_sphereAndShapeContactBlocks;

	b3SoftBody* m_body;
	b3List<b3SoftBodySphereAndShapeContact> m_sphereAndShapeContactList;
};

#endif
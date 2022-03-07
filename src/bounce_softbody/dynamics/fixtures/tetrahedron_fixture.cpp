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

#include <bounce_softbody/dynamics/fixtures/tetrahedron_fixture.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/body.h>

b3TetrahedronFixture::b3TetrahedronFixture(const b3TetrahedronFixtureDef& def, b3Body* body) : b3Fixture(def, body)
{
	m_type = e_tetrahedronFixture;
	
	m_p1 = def.p1;
	m_p2 = def.p2;
	m_p3 = def.p3;
	m_p4 = def.p4;

	b3Vec3 A = def.v1;
	b3Vec3 B = def.v2;
	b3Vec3 C = def.v3;
	b3Vec3 D = def.v4;

	b3Vec3 E1 = B - A;
	b3Vec3 E2 = C - A;
	b3Vec3 E3 = D - A;

	const scalar inv6 = scalar(1) / scalar(6);

	scalar det = b3Det(E1, E2, E3);
	scalar sign = b3Sign(det);
	m_volume = inv6 * sign * det;

	m_prev = nullptr;
	m_next = nullptr;
}
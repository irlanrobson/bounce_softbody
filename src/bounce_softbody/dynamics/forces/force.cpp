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

#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/dynamics/forces/stretch_force.h>
#include <bounce_softbody/dynamics/forces/shear_force.h>
#include <bounce_softbody/dynamics/forces/spring_force.h>
#include <bounce_softbody/dynamics/forces/mouse_force.h>
#include <bounce_softbody/dynamics/forces/triangle_element_force.h>
#include <bounce_softbody/dynamics/forces/tetrahedron_element_force.h>
#include <bounce_softbody/common/memory/block_allocator.h>

b3Force* b3Force::Create(const b3ForceDef* def, b3BlockAllocator* allocator)
{
	b3Force* force = nullptr;
	switch (def->type)
	{
	case e_stretchForce:
	{
		void* mem = allocator->Allocate(sizeof(b3StretchForce));
		force = new (mem) b3StretchForce((b3StretchForceDef*)def);
		break;
	}
	case e_shearForce:
	{
		void* mem = allocator->Allocate(sizeof(b3ShearForce));
		force = new (mem) b3ShearForce((b3ShearForceDef*)def);
		break;
	}
	case e_springForce:
	{
		void* mem = allocator->Allocate(sizeof(b3SpringForce));
		force = new (mem) b3SpringForce((b3SpringForceDef*)def);
		break;
	}
	case e_mouseForce:
	{
		void* mem = allocator->Allocate(sizeof(b3MouseForce));
		force = new (mem) b3MouseForce((b3MouseForceDef*)def);
		break;
	}
	case e_triangleElementForce:
	{
		void* mem = allocator->Allocate(sizeof(b3TriangleElementForce));
		force = new (mem) b3TriangleElementForce((b3TriangleElementForceDef*)def);
		break;
	}
	case e_tetrahedronElementForce:
	{
		void* mem = allocator->Allocate(sizeof(b3TetrahedronElementForce));
		force = new (mem) b3TetrahedronElementForce((b3TetrahedronElementForceDef*)def);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	return force;
}

void b3Force::Destroy(b3Force* force, b3BlockAllocator* allocator)
{
	B3_ASSERT(force);

	b3ForceType type = force->GetType();
	switch (type)
	{
	case e_stretchForce:
	{
		b3StretchForce* o = (b3StretchForce*)force;
		o->~b3StretchForce();
		allocator->Free(o, sizeof(b3StretchForce));
		break;
	}
	case e_shearForce:
	{
		b3ShearForce* o = (b3ShearForce*)force;
		o->~b3ShearForce();
		allocator->Free(o, sizeof(b3ShearForce));
		break;
	}
	case e_springForce:
	{
		b3SpringForce* o = (b3SpringForce*)force;
		o->~b3SpringForce();
		allocator->Free(o, sizeof(b3SpringForce));
		break;
	}
	case e_mouseForce:
	{
		b3MouseForce* o = (b3MouseForce*)force;
		o->~b3MouseForce();
		allocator->Free(o, sizeof(b3MouseForce));
		break;
	}
	case e_triangleElementForce:
	{
		b3TriangleElementForce* o = (b3TriangleElementForce*)force;
		o->~b3TriangleElementForce();
		allocator->Free(o, sizeof(b3TriangleElementForce));
		break;
	}
	case e_tetrahedronElementForce:
	{
		b3TetrahedronElementForce* o = (b3TetrahedronElementForce*)force;
		o->~b3TetrahedronElementForce();
		allocator->Free(o, sizeof(b3TetrahedronElementForce));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	};
}

b3Force::b3Force()
{
	m_type = e_unknownForce;
	m_meshIndex = B3_MAX_U32;
	m_prev = nullptr;
	m_next = nullptr;
}
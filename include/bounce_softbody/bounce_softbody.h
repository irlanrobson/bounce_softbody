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

#ifndef BOUNCE_SOFTBODY_H
#define BOUNCE_SOFTBODY_H

// These files constitute the main Bounce Softbody API.

#include <bounce_softbody/common/settings.h>
#include <bounce_softbody/common/draw.h>

#include <bounce_softbody/collision/geometry/mesh.h>
#include <bounce_softbody/collision/geometry/grid_mesh.h>
#include <bounce_softbody/collision/geometry/sphere_mesh.h>
#include <bounce_softbody/collision/geometry/sdf.h>

#include <bounce_softbody/collision/shapes/sphere_shape.h>
#include <bounce_softbody/collision/shapes/capsule_shape.h>
#include <bounce_softbody/collision/shapes/triangle_shape.h>
#include <bounce_softbody/collision/shapes/box_shape.h>
#include <bounce_softbody/collision/shapes/mesh_shape.h>
#include <bounce_softbody/collision/shapes/sdf_shape.h>

#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/dynamics/particle.h>

#include <bounce_softbody/dynamics/forces/stretch_force.h>
#include <bounce_softbody/dynamics/forces/shear_force.h>
#include <bounce_softbody/dynamics/forces/spring_force.h>
#include <bounce_softbody/dynamics/forces/mouse_force.h>
#include <bounce_softbody/dynamics/forces/triangle_element_force.h>
#include <bounce_softbody/dynamics/forces/tetrahedron_element_force.h>

#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/triangle_fixture.h>
#include <bounce_softbody/dynamics/fixtures/tetrahedron_fixture.h>
#include <bounce_softbody/dynamics/fixtures/world_fixture.h>

#endif

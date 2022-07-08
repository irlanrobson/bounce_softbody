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

#ifndef B3_MESH_SHAPE_H
#define B3_MESH_SHAPE_H

#include <bounce_softbody/collision/shapes/shape.h>

struct b3Mesh;
class b3TriangleShape;

// A triangle mesh shape.
// You must either provide a positive radius to the particle spheres or to this shape (or both) to 
// support collisions with the shape. 
class b3MeshShape : public b3Shape
{
public:
	b3MeshShape();
	
	b3Shape* Clone(b3BlockAllocator* allocator) const;

	b3AABB ComputeAABB() const;

	bool Collide(b3SphereManifold* manifold, const b3Sphere& sphere) const;

	void GetChildTriangle(b3TriangleShape* triangle, uint32 childIndex) const;
	
	void Draw(b3Draw* draw) const;

	// The shared mesh pointer. This must remain in scope while the shape is in use.
	const b3Mesh* m_mesh;

	// Transform.
	b3Transform m_xf;

	// Scale. It can be non-uniform and negative.
	b3Vec3 m_scale;
};

#endif
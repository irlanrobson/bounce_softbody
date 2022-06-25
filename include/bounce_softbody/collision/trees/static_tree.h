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

#ifndef B3_STATIC_TREE_H
#define B3_STATIC_TREE_H

#include <bounce_softbody/common/template/stack.h>
#include <bounce_softbody/collision/collision.h>
#include <bounce_softbody/collision/geometry/aabb.h>
#include <bounce_softbody/collision/geometry/sphere.h>

#define B3_NULL_STATIC_NODE B3_MAX_U32

class b3Draw;

// A node in a static tree. The client does not interact with this directly.
struct b3StaticNode
{
	// Is this node a leaf?
	bool IsLeaf() const
	{
		return child1 == B3_NULL_STATIC_NODE;
	}

	b3AABB aabb;
	
	uint32 parent;

	uint32 child1;
	union
	{
		uint32 child2;
		uint32 index;
	};
};

// AABB tree for static AABBs.
class b3StaticTree
{
public:
	b3StaticTree();
	~b3StaticTree();

	// Build this tree from an array of AABBs.
	void Build(const b3AABB* aabbs, uint32 count);

	// Get the AABB of a given node.
	const b3AABB& GetAABB(uint32 nodeId) const;

	// Get the user index associated with a given node.
	uint32 GetIndex(uint32 nodeId) const;

	// Report the client callback all AABBs that are overlapping with
	// the given AABB. The client must return false to cancel the query 
	// or true to continue the query.
	template<class T>
	void QueryAABB(T* callback, const b3AABB& aabb) const;

	// Report the client callback all AABBs that are overlapping with
	// the given sphere. The client must return false to cancel the query 
	// or true to continue the query.
	template<class T>
	void QuerySphere(T* callback, const b3Sphere& sphere) const;
	
	// Report the client callback all AABBs that are overlapping with
	// the given ray. The client callback must return the new intersection fraction. 
	// If fraction == 0 then the ray-cast is cancelled immediatly. Otherwise the ray is clipped 
	// to the new fraction.
	template<class T>
	void RayCast(T* callback, const b3RayCastInput& input) const;

	// Draw this tree.
	void Draw(b3Draw* draw) const;
private:
	// Build a node. Return the node index.
	uint32 BuildNode(uint32 parentId, const b3AABB* aabbs, uint32* indices, uint32 count);

	b3StaticNode* m_nodes;
	uint32 m_leafCapacity;
	uint32 m_leafCount;
	uint32 m_internalCapacity;
	uint32 m_internalCount;
	uint32 m_nodeCapacity;
	uint32 m_nodeCount;
	uint32 m_root;
};

inline const b3AABB& b3StaticTree::GetAABB(uint32 nodeId) const
{
	B3_ASSERT(nodeId < m_nodeCount);
	return m_nodes[nodeId].aabb;
}

inline uint32 b3StaticTree::GetIndex(uint32 nodeId) const
{
	B3_ASSERT(nodeId < m_nodeCount);
	B3_ASSERT(m_nodes[nodeId].IsLeaf());
	return m_nodes[nodeId].index;
}

template<class T>
inline void b3StaticTree::QueryAABB(T* callback, const b3AABB& aabb) const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false)
	{
		uint32 nodeIndex = stack.Top();

		stack.Pop();

		if (nodeIndex == B3_NULL_STATIC_NODE)
		{
			continue;
		}

		const b3StaticNode* node = m_nodes + nodeIndex;

		if (b3TestOverlap(node->aabb, aabb) == true)
		{
			if (node->IsLeaf() == true)
			{
				if (callback->Report(nodeIndex) == false)
				{
					return;
				}
			}
			else
			{
				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}
}

template<class T>
inline void b3StaticTree::QuerySphere(T* callback, const b3Sphere& sphere) const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false)
	{
		uint32 nodeIndex = stack.Top();

		stack.Pop();

		if (nodeIndex == B3_NULL_STATIC_NODE)
		{
			continue;
		}

		const b3StaticNode* node = m_nodes + nodeIndex;

		if (b3TestOverlap(node->aabb, sphere) == true)
		{
			if (node->IsLeaf() == true)
			{
				if (callback->Report(nodeIndex) == false)
				{
					return;
				}
			}
			else
			{
				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}
}

template<class T>
inline void b3StaticTree::RayCast(T* callback, const b3RayCastInput& input) const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Vec3 p1 = input.p1;
	b3Vec3 p2 = input.p2;
	b3Vec3 r = p2 - p1;
	B3_ASSERT(b3LengthSquared(r) > scalar(0));
	r.Normalize();

	scalar maxFraction = input.maxFraction;

	// Build an AABB for the segment.
	b3Vec3 q2;
	b3AABB segmentAABB;
	{
		q2 = p1 + maxFraction * (p2 - p1);
		segmentAABB.lowerBound = b3Min(p1, q2);
		segmentAABB.upperBound = b3Max(p1, q2);
	}

	b3Vec3 e1 = b3Vec3_x;
	b3Vec3 e2 = b3Vec3_y;
	b3Vec3 e3 = b3Vec3_z;

	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false)
	{
		uint32 nodeIndex = stack.Top();
		
		stack.Pop();

		if (nodeIndex == B3_NULL_STATIC_NODE)
		{
			continue;
		}

		const b3StaticNode* node = m_nodes + nodeIndex;

		if (b3TestOverlap(segmentAABB, node->aabb) == false)
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		b3Vec3 c = node->aabb.GetCenter();
		b3Vec3 h = node->aabb.GetExtents();

		b3Vec3 s = p1 - c;
		b3Vec3 t = q2 - c;

		// |sigma + tau| > |sigma - tau| + 2 * eta
		scalar sigma_1 = s.x;
		scalar tau_1 = t.x;
		scalar eta_1 = h.x;

		scalar s1 = b3Abs(sigma_1 + tau_1) - (b3Abs(sigma_1 - tau_1) + scalar(2) * eta_1);
		if (s1 > scalar(0))
		{
			continue;
		}

		scalar sigma_2 = s.y;
		scalar tau_2 = t.y;
		scalar eta_2 = h.y;

		scalar s2 = b3Abs(sigma_2 + tau_2) - (b3Abs(sigma_2 - tau_2) + scalar(2) * eta_2);
		if (s2 > scalar(0))
		{
			continue;
		}

		scalar sigma_3 = s.z;
		scalar tau_3 = t.z;
		scalar eta_3 = h.z;

		scalar s3 = b3Abs(sigma_3 + tau_3) - (b3Abs(sigma_3 - tau_3) + scalar(2) * eta_3);
		if (s3 > scalar(0))
		{
			continue;
		}

		// v = cross(ei, r)
		// |dot(v, s)| > dot(|v|, h)
		b3Vec3 v1 = b3Cross(e1, r);
		b3Vec3 abs_v1 = b3Abs(v1);
		scalar s4 = b3Abs(b3Dot(v1, s)) - b3Dot(abs_v1, h);
		if (s4 > scalar(0))
		{
			continue;
		}

		b3Vec3 v2 = b3Cross(e2, r);
		b3Vec3 abs_v2 = b3Abs(v2);
		scalar s5 = b3Abs(b3Dot(v2, s)) - b3Dot(abs_v2, h);
		if (s5 > scalar(0))
		{
			continue;
		}

		b3Vec3 v3 = b3Cross(e3, r);
		b3Vec3 abs_v3 = b3Abs(v3);
		scalar s6 = b3Abs(b3Dot(v3, s)) - b3Dot(abs_v3, h);
		if (s6 > scalar(0))
		{
			continue;
		}

		if (node->IsLeaf() == true)
		{
			b3RayCastInput subInput;
			subInput.p1 = input.p1;
			subInput.p2 = input.p2;
			subInput.maxFraction = maxFraction;

			scalar newMaxFraction = callback->Report(subInput, nodeIndex);

			if (newMaxFraction == scalar(0))
			{
				// The client has stopped the query.
				return;
			}

			if (newMaxFraction > scalar(0))
			{
				// Update the segment AABB.
				maxFraction = newMaxFraction;
				q2 = p1 + maxFraction * (p2 - p1);
				segmentAABB.lowerBound = b3Min(p1, q2);
				segmentAABB.upperBound = b3Max(p1, q2);
			}
		}
		else
		{
			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}

#endif
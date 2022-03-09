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

#ifndef B3_DYNAMIC_TREE_H
#define B3_DYNAMIC_TREE_H

#include <bounce_softbody/common/template/stack.h>
#include <bounce_softbody/collision/geometry/aabb.h>

#define B3_NULL_DYNAMIC_NODE B3_MAX_U32

class b3Draw;

// A node in a dynamic tree. The client does not interact with this directly.
struct b3DynamicNode
{
	// Is this node a leaf?
	bool IsLeaf() const
	{
		//A node is a leaf if child 2 == B3_NULL_DYNAMIC_NODE or height == 0.
		return child1 == B3_NULL_DYNAMIC_NODE;
	}

	// The fattened node AABB.
	b3AABB aabb;

	// The associated user data.
	void* userData;

	union
	{
		uint32 parent;
		uint32 next;
	};

	uint32 child1;
	uint32 child2;

	// Leaf if 0, free node if -1
	int32 height;
};

// A dynamic AABB tree , inspired by Erin Catto's b3DynamicTree.
// A dynamic tree stores data in a binary tree to accelerate
// queries such as volume queries and ray casts. Leafs are proxies
// with an AABB. In the tree we expand the proxy AABB by B3_AABB_ETENSION
// so that the proxy AABB is larger than the client object. 
// This allows the client object to move by small amounts without triggering 
// a tree update.
// 
// Nodes are pooled and relocatable, so we use node indices rather than pointers.
class b3DynamicTree
{
public:
	b3DynamicTree();
	~b3DynamicTree();

	// Create a proxy. Give it a tight fitting AABB and user pointer.
	uint32 CreateProxy(const b3AABB& aabb, void* userData);

	// Destroy a given proxy.
	void DestroyProxy(uint32 proxyId);

	// Update an existing proxy AABB with a given AABB and a displacement.
	// displacement = dt * velocity
	// Return true if the proxy has moved.
	bool MoveProxy(uint32 proxyId, const b3AABB& aabb, const b3Vec3& displacement);

	// Get the fat AABB of a given proxy.
	const b3AABB& GetFatAABB(uint32 proxyId) const;

	// Get the data associated with a given proxy.
	void* GetUserData(uint32 proxyId) const;

	// Check if two aabbs in this tree are overlapping.
	bool TestOverlap(uint32 proxy1, uint32 proxy2) const;

	// Keep reporting the client callback the AABBs that are overlapping with
	// the given AABB. The client callback must return true if the query 
	// must be stopped or false to continue looking for more overlapping pairs.
	template<class T>
	void QueryAABB(T* callback, const b3AABB& aabb) const;

	// Keep reporting the client callback all AABBs that are overlapping with
	// the given ray. The client callback must return the new intersection fraction.
	// If the fraction == 0 then the query is cancelled immediately.
	template<class T>
	void RayCast(T* callback, const b3RayCastInput& input) const;

	// Draw this tree.
	void Draw(b3Draw* draw) const;
private:
	// Insert a node into the tree.
	void InsertLeaf(uint32 node);

	// Remove a node from the tree.
	void RemoveLeaf(uint32 node);

	// Rebuild the hierarchy starting from the given node.
	void Refit(uint32 node);

	// Pick the best node that can be merged with a given AABB.
	uint32 PickBest(const b3AABB& aabb) const;

	// Peel a node from the free list and insert into the node array. 
	// Allocate a new node if necessary. The function returns the new node index.
	uint32 AllocateNode();

	// Free a node from the node pool and add it to the free list.
	void FreeNode(uint32 node);

	// Make a node available for the next allocation.
	void AddToFreeList(uint32 node);

	// Balance the tree.
	uint32 Balance(uint32 index);

	// Validate a given node of this tree.
	void Validate(uint32 node) const;

	// The root of this tree.
	uint32 m_root;

	// The nodes of this tree stored in an array.
	b3DynamicNode* m_nodes;
	uint32 m_nodeCount;
	uint32 m_nodeCapacity;
	uint32 m_freeList;
};

inline const b3AABB& b3DynamicTree::GetFatAABB(uint32 proxyId) const
{
	B3_ASSERT(proxyId != B3_NULL_DYNAMIC_NODE && proxyId < m_nodeCapacity);
	return m_nodes[proxyId].aabb;
}

inline void* b3DynamicTree::GetUserData(uint32 proxyId) const
{
	B3_ASSERT(proxyId != B3_NULL_DYNAMIC_NODE && proxyId < m_nodeCapacity);
	return m_nodes[proxyId].userData;
}

inline bool b3DynamicTree::TestOverlap(uint32 proxy1, uint32 proxy2) const
{
	B3_ASSERT(proxy1 != B3_NULL_DYNAMIC_NODE && proxy1 < m_nodeCapacity);
	B3_ASSERT(proxy2 != B3_NULL_DYNAMIC_NODE && proxy2 < m_nodeCapacity);
	return b3TestOverlap(m_nodes[proxy1].aabb, m_nodes[proxy2].aabb);
}

template<class T>
inline void b3DynamicTree::QueryAABB(T* callback, const b3AABB& aabb) const
{
	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false)
	{
		uint32 nodeIndex = stack.Top();
		stack.Pop();

		if (nodeIndex == B3_NULL_DYNAMIC_NODE)
		{
			continue;
		}

		const b3DynamicNode* node = m_nodes + nodeIndex;

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
inline void b3DynamicTree::RayCast(T* callback, const b3RayCastInput& input) const
{
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

		if (nodeIndex == B3_NULL_DYNAMIC_NODE)
		{
			continue;
		}

		const b3DynamicNode* node = m_nodes + nodeIndex;

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
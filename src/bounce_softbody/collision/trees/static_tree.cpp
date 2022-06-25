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

#include <bounce_softbody/collision/trees/static_tree.h>
#include <bounce_softbody/common/draw.h>
#include <algorithm>

b3StaticTree::b3StaticTree()
{
	m_root = B3_NULL_STATIC_NODE;
	m_nodes = nullptr;
	m_leafCapacity = 0;
	m_leafCount = 0;
	m_internalCapacity = 0;
	m_internalCount = 0;
	m_nodeCapacity = 0;
	m_nodeCount = 0;
}

b3StaticTree::~b3StaticTree()
{
	b3Free(m_nodes);
}

struct b3SortPredicate
{
	bool operator()(uint32 i1, uint32 i2)
	{
		b3Vec3 c1 = aabbs[i1].GetCenter();
		b3Vec3 c2 = aabbs[i2].GetCenter();
		return c1[axis] < c2[axis];
	}

	const b3AABB* aabbs;
	uint32 axis;
};

static uint32 b3Partition(const b3AABB& aabb, const b3AABB* aabbs, uint32* indices, uint32 count)
{
	// Choose a partitioning axis.
	uint32 splitAxis = aabb.GetLongestAxis();

	// Sort indices along the split axis.
	b3SortPredicate predicate;
	predicate.aabbs = aabbs;
	predicate.axis = splitAxis;

	std::sort(indices, indices + count, predicate);

	// Choose a split point.
	scalar splitPos = aabb.GetCenter()[splitAxis];

	// Find the AABB that splits the set in two subsets.
	uint32 k = 0;
	while (k < count)
	{
		uint32 index = indices[k];
		
		b3Vec3 center = aabbs[index].GetCenter();
		
		if (center[splitAxis] > splitPos)
		{
			break;
		}
		
		++k;
	}

	// Ensure nonempty subsets.
	if (k == 0 || k == count)
	{
		// Choose median.
		k = count / 2;
	}

	return k;
}

uint32 b3StaticTree::BuildNode(uint32 parentId, const b3AABB* aabbs, uint32* indices, uint32 count)
{
	B3_ASSERT(count > 0);

	B3_ASSERT(m_nodeCount < m_nodeCapacity);
	uint32 nodeId = m_nodeCount;
	++m_nodeCount;

	b3StaticNode* node = m_nodes + nodeId;
	node->parent = parentId;

	if (count <= 1)
	{
		B3_ASSERT(m_leafCount < m_leafCapacity);
		++m_leafCount;
		
		// Set node as leaf
		node->aabb = aabbs[indices[0]];
		node->child1 = B3_NULL_STATIC_NODE;
		node->index = indices[0];
	}
	else
	{
		B3_ASSERT(m_internalCount < m_internalCapacity);
		++m_internalCount;

		// Compute node AABB
		b3AABB aabb = aabbs[indices[0]];
		for (uint32 i = 1; i < count; ++i)
		{
			aabb.Combine(aabbs[indices[i]]);
		}

		node->aabb = aabb;

		// Partition boxes
		uint32 k = b3Partition(aabb, aabbs, indices, count);

		// Build children
		node->child1 = BuildNode(nodeId, aabbs, indices, k);
		node->child2 = BuildNode(nodeId, aabbs, indices + k, count - k);
	}

	return nodeId;
}

void b3StaticTree::Build(const b3AABB* aabbs, uint32 count)
{
	// This function should be called once.
	B3_ASSERT(m_nodes == nullptr && m_nodeCount == 0);
	B3_ASSERT(count > 0);

	// Leafs = n, Internals = n - 1, Total = 2n - 1, if we assume
	// each leaf node contains exactly 1 object.
	m_leafCapacity = count;
	m_internalCapacity = count - 1;
	m_nodeCapacity = 2 * count - 1;

	m_nodes = (b3StaticNode*)b3Alloc(m_nodeCapacity * sizeof(b3StaticNode));
	
	uint32* indices = (uint32*)b3Alloc(count * sizeof(uint32));
	for (uint32 i = 0; i < count; ++i)
	{
		indices[i] = i;
	}

	m_root = BuildNode(B3_NULL_STATIC_NODE, aabbs, indices, count);

	b3Free(indices);

	B3_ASSERT(m_leafCount == m_leafCapacity);
	B3_ASSERT(m_internalCount == m_internalCapacity);
	B3_ASSERT(m_nodeCount == m_nodeCapacity);
}

void b3StaticTree::Draw(b3Draw* draw) const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (!stack.IsEmpty())
	{
		uint32 nodeIndex = stack.Top();
		stack.Pop();

		if (nodeIndex == B3_NULL_STATIC_NODE)
		{
			continue;
		}

		const b3StaticNode* node = m_nodes + nodeIndex;
		if (node->IsLeaf())
		{
			draw->DrawAABB(node->aabb, b3Color_red);
		}
		else
		{
			draw->DrawAABB(node->aabb, b3Color_green);

			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}
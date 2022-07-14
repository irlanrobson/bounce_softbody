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

void b3StaticTree::Draw(b3Draw* draw) const
{
	if (nodeCount == 0)
	{
		return;
	}

	b3Stack<uint32, 256> stack;
	stack.Push(root);

	while (!stack.IsEmpty())
	{
		uint32 nodeIndex = stack.Top();
		stack.Pop();

		if (nodeIndex == B3_NULL_STATIC_NODE)
		{
			continue;
		}

		const b3StaticNode* node = nodes + nodeIndex;
		
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

static uint32 b3BuildNode(b3StaticTree* tree, uint32 nodeCapacity,
	uint32 parentIndex, const b3AABB* aabbs, uint32* indices, uint32 count)
{
	B3_ASSERT(count > 0);

	B3_ASSERT(tree->nodeCount < nodeCapacity);
	uint32 nodeIndex = tree->nodeCount;
	++tree->nodeCount;

	b3StaticNode* node = tree->nodes + nodeIndex;
	node->parent = parentIndex;

	if (count == 1)
	{
		// Node is leaf
		node->aabb = aabbs[indices[0]];
		node->child1 = B3_NULL_STATIC_NODE;
		node->index = indices[0];
	}
	else
	{
		// Node is internal

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
		node->child1 = b3BuildNode(tree, nodeCapacity, nodeIndex, aabbs, indices, k);
		node->child2 = b3BuildNode(tree, nodeCapacity, nodeIndex, aabbs, indices + k, count - k);
	}

	return nodeIndex;
}

void b3BuildTree(b3StaticTree* tree, const b3AABB* aabbs, uint32 count)
{
	// This function should be called only once for each tree.
	B3_ASSERT(tree->nodes == nullptr && tree->nodeCount == 0);
	B3_ASSERT(count > 0);

	// Leafs = n, Internals = n - 1, Total = 2n - 1, if we assume
	// each leaf node contains exactly 1 object.
	uint32 nodeCapacity = 2 * count - 1;
	
	tree->nodes = (b3StaticNode*)b3Alloc(nodeCapacity * sizeof(b3StaticNode));
	
	uint32* indices = (uint32*)b3Alloc(count * sizeof(uint32));
	for (uint32 i = 0; i < count; ++i)
	{
		indices[i] = i;
	}
	
	// Build
	tree->root = b3BuildNode(tree, nodeCapacity, B3_NULL_STATIC_NODE, aabbs, indices, count);

	b3Free(indices);
	
	B3_ASSERT(tree->nodeCount == nodeCapacity);
}

void b3DestroyTree(b3StaticTree* tree)
{
	b3Free(tree->nodes);
}
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

#include <bounce_softbody/common/memory/block_allocator.h>
#include <bounce_softbody/common/memory/block_pool.h>
#include <new>

static const uint32 b3_maxBlockSize = 640;

// These are the supported object sizes. Actual allocations are rounded up the next size.
static const uint32 b3_blockSizes[b3_blockSizeCount] =
{
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
};

// This structure maps an arbitrary allocation size to a suitable slot in b3_blockSizes.
struct b3SizeMap
{
	b3SizeMap()
	{
		uint32 j = 0;
		slots[0] = 0;
		for (uint32 i = 1; i <= b3_maxBlockSize; ++i)
		{
			B3_ASSERT(j < b3_blockSizeCount);
			if (i <= b3_blockSizes[j])
			{
				slots[i] = (uint8)j;
			}
			else
			{
				++j;
				slots[i] = (uint8)j;
			}
		}
	}

	uint8 slots[b3_maxBlockSize + 1];
};

static const b3SizeMap b3_sizeMap;

b3BlockAllocator::b3BlockAllocator()
{
	m_blockPools = (b3BlockPool*)b3Alloc(sizeof(b3BlockPool) * b3_blockSizeCount);
	for (uint32 i = 0; i < b3_blockSizeCount; ++i)
	{
		new (m_blockPools + i) b3BlockPool(b3_blockSizes[i]);
	}
}

b3BlockAllocator::~b3BlockAllocator()
{
	for (uint32 i = 0; i < b3_blockSizeCount; ++i)
	{
		m_blockPools[i].~b3BlockPool();
	}
	b3Free(m_blockPools);
}

void* b3BlockAllocator::Allocate(uint32 size)
{
	if (size == 0)
	{
		return nullptr;
	}

	B3_ASSERT(0 < size);
	
	if (size > b3_maxBlockSize)
	{
		return b3Alloc(size);
	}

	uint32 index = b3_sizeMap.slots[size];
	B3_ASSERT(0 <= index && index < b3_blockSizeCount);

	return m_blockPools[index].Allocate();
}

void b3BlockAllocator::Free(void* p, uint32 size)
{
	if (size == 0)
	{
		return;
	}

	B3_ASSERT(0 < size);

	if (size > b3_maxBlockSize)
	{
		b3Free(p);
		return;
	}
	
	uint32 index = b3_sizeMap.slots[size];
	B3_ASSERT(0 <= index && index < b3_blockSizeCount);

	m_blockPools[index].Free(p);
}
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

#ifndef B3_BLOCK_ALLOCATOR_H
#define B3_BLOCK_ALLOCATOR_H

#include <bounce_softbody/common/settings.h>

class b3BlockPool;

// Number of blocks pools.
const uint32 b3_blockSizeCount = 14;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class b3BlockAllocator
{
public:
	b3BlockAllocator();
	~b3BlockAllocator();

	// Allocate memory. This will use b3Alloc if the size is larger than b3_maxBlockSize.
	void* Allocate(uint32 size);

	// Free memory. This will use b3Free if the size is larger than b3_maxBlockSize.
	void Free(void* p, uint32 size);
private:
	// One pool per block size.
	b3BlockPool* m_blockPools;
};

#endif
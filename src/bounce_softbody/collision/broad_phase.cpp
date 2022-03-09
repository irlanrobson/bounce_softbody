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

#include <bounce_softbody/collision/broad_phase.h>

b3BroadPhase::b3BroadPhase() 
{
	m_proxyCount = 0;

	m_moveBufferCapacity = 16;
	m_moveBuffer = (uint32*)b3Alloc(m_moveBufferCapacity * sizeof(uint32));
	memset(m_moveBuffer, 0, m_moveBufferCapacity * sizeof(uint32));
	m_moveBufferCount = 0;

	m_pairCapacity = 16;
	m_pairs = (b3Pair*)b3Alloc(m_pairCapacity * sizeof(b3Pair));
	memset(m_pairs, 0, m_pairCapacity * sizeof(b3Pair));
	m_pairCount = 0;
}

b3BroadPhase::~b3BroadPhase() 
{
	b3Free(m_moveBuffer);
	b3Free(m_pairs);
}

void b3BroadPhase::BufferMove(uint32 proxyId) 
{
	// The proxy has been moved. Add it to the buffer of moved proxies.
	// Check capacity.
	if (m_moveBufferCount == m_moveBufferCapacity) 
	{
		// Duplicate capacity.
		m_moveBufferCapacity *= 2;

		uint32* oldMoveBuffer = m_moveBuffer;
		m_moveBuffer = (uint32*)b3Alloc(m_moveBufferCapacity * sizeof(uint32));
		memcpy(m_moveBuffer, oldMoveBuffer, m_moveBufferCount * sizeof(uint32));
		b3Free(oldMoveBuffer);
	}

	// Add to move buffer.
	m_moveBuffer[m_moveBufferCount] = proxyId;
	++m_moveBufferCount;
}

void b3BroadPhase::UnbufferMove(uint32 proxyId)
{
	for (uint32 i = 0; i < m_moveBufferCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = B3_NULL_PROXY;
		}
	}
}

bool b3BroadPhase::TestOverlap(uint32 proxy1, uint32 proxy2) const 
{
	return m_tree.TestOverlap(proxy1, proxy2);
}

uint32 b3BroadPhase::CreateProxy(const b3AABB& aabb, void* userData) 
{
	uint32 proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void b3BroadPhase::DestroyProxy(uint32 proxyId) 
{
	UnbufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

void b3BroadPhase::MoveProxy(uint32 proxyId, const b3AABB& aabb, const b3Vec3& displacement)
{
	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		// Buffer the moved proxy.
		BufferMove(proxyId);
	}
}

void b3BroadPhase::TouchProxy(uint32 proxyId)
{
	BufferMove(proxyId);
}

bool b3BroadPhase::Report(uint32 proxyId) 
{
	if (proxyId == m_queryProxyId) 
	{
		// The proxy can't overlap with itself.
		return true;
	}

	// Check capacity.
	if (m_pairCount == m_pairCapacity) 
	{
		// Duplicate capacity.
		m_pairCapacity *= 2;
		
		b3Pair* oldPairs = m_pairs;
		m_pairs = (b3Pair*)b3Alloc(m_pairCapacity * sizeof(b3Pair));
		memcpy(m_pairs, oldPairs, m_pairCount * sizeof(b3Pair));
		b3Free(oldPairs);
	}

	// Add overlapping pair to the pair buffer.
	m_pairs[m_pairCount].proxy1 = b3Min(proxyId, m_queryProxyId);
	m_pairs[m_pairCount].proxy2 = b3Max(proxyId, m_queryProxyId);
	++m_pairCount;

	// Keep looking for overlapping pairs.
	return true;
}
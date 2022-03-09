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

#ifndef B3_BROAD_PHASE_H
#define B3_BROAD_PHASE_H

#include <bounce_softbody/collision/trees/dynamic_tree.h>
#include <algorithm>

#define B3_NULL_PROXY B3_MAX_U32

// A pair of broad-phase proxies.
struct b3Pair
{
	uint32 proxy1;
	uint32 proxy2;
};

// The broad-phase interface. 
// It is used to perform ray casts, volume queries, and overlapping queries 
// against AABBs.
class b3BroadPhase 
{
public:
	b3BroadPhase();
	~b3BroadPhase();

	// Create a proxy and return a index to it.
	uint32 CreateProxy(const b3AABB& aabb, void* userData);
	
	// Destroy a given proxy and remove it from the broadphase.
	void DestroyProxy(uint32 proxyId);

	// Update an existing proxy AABB with a given AABB and a displacement.
	// displacement = dt * velocity
	void MoveProxy(uint32 proxyId, const b3AABB& aabb, const b3Vec3& displacement);

	// Force move the proxy.
	void TouchProxy(uint32 proxyId);

	// Get the fat AABB of a given proxy.
	const b3AABB& GetFatAABB(uint32 proxyId) const;

	// Get the user data attached to a proxy.
	void* GetUserData(uint32 proxyId) const;

	// Get the number of proxies.
	uint32 GetProxyCount() const;

	// Test if two proxy AABBs are overlapping.
	bool TestOverlap(uint32 proxy1, uint32 proxy2) const;
	
	// Notify the client callback the AABBs that are overlapping with the passed AABB.
	template<class T>
	void QueryAABB(T* callback, const b3AABB& aabb) const;
	
	// Notify the client callback the AABBs that are overlapping the 
	// passed ray.
	template<class T>
	void RayCast(T* callback, const b3RayCastInput& input) const;

	// Find and store overlapping AABB pairs.
	// Notify the client callback the AABB pairs that are overlapping.
	// The client must store the notified pairs.
	template<class T>
	void FindPairs(T* callback);

	// Draw the proxy AABBs.
	void Draw(b3Draw* draw) const;
private :
	friend class b3DynamicTree;

	void BufferMove(uint32 proxyId);
	void UnbufferMove(uint32 proxyId);
	
	// The client callback used to add an overlapping pair
	// to the overlapping pair buffer.
	bool Report(uint32 proxyId);
	
	// The dynamic tree.
	b3DynamicTree m_tree;

	// Number of proxies
	uint32 m_proxyCount;

	// The current proxy being queried for overlap with another proxies. 
	// It is used to avoid a proxy overlap with itself.
	uint32 m_queryProxyId;

	// The objects that have moved in a step.
	uint32* m_moveBuffer;
	uint32 m_moveBufferCount;
	uint32 m_moveBufferCapacity;

	// The buffer holding the unique overlapping AABB pairs.
	b3Pair* m_pairs;
	uint32 m_pairCapacity;
	uint32 m_pairCount;
};

inline const b3AABB& b3BroadPhase::GetFatAABB(uint32 proxyId) const 
{
	return m_tree.GetFatAABB(proxyId);
}

inline void* b3BroadPhase::GetUserData(uint32 proxyId) const 
{
	return m_tree.GetUserData(proxyId);
}

inline uint32 b3BroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}

template<class T>
inline void b3BroadPhase::QueryAABB(T* callback, const b3AABB& aabb) const 
{
	return m_tree.QueryAABB(callback, aabb);
}

template<class T>
inline void b3BroadPhase::RayCast(T* callback, const b3RayCastInput& input) const 
{
	return m_tree.RayCast(callback, input);
}

static B3_FORCE_INLINE bool operator<(const b3Pair& pair1, const b3Pair& pair2) 
{
	if (pair1.proxy1 < pair2.proxy1) 
	{
		return true;
	}

	if (pair1.proxy1 == pair2.proxy1) 
	{
		return pair1.proxy2 < pair2.proxy2;
	}

	return false;
}

template<class T>
inline void b3BroadPhase::FindPairs(T* callback) 
{
	// Reset the overlapping pairs buffer count for the current step.
	m_pairCount = 0;

	// Notifying this class with QueryCallback(), gets the (duplicated) overlapping pair buffer.
	for (uint32 i = 0; i < m_moveBufferCount; ++i) 
	{
		// Keep the current queried proxy ID to avoid self overlapping.
		m_queryProxyId = m_moveBuffer[i];

		if (m_queryProxyId == B3_NULL_PROXY)
		{
			// Proxy was unbuffered
			continue;
		}

		const b3AABB& aabb = m_tree.GetAABB(m_queryProxyId);
		m_tree.QueryAABB(this, aabb);
	}

	// Reset the move buffer for the next step.
	m_moveBufferCount = 0;

	// Sort the (duplicated) overlapping pair buffer to prune duplicated pairs.
	std::sort(m_pairs, m_pairs + m_pairCount);

	// Skip duplicated overlapping pairs.
	uint32 index = 0;
	while (index < m_pairCount) 
	{
		const b3Pair* primaryPair = m_pairs + index;

		// Report an unique overlapping pair to the client.
		callback->AddPair(m_tree.GetUserData(primaryPair->proxy1), m_tree.GetUserData(primaryPair->proxy2));

		// Skip all duplicated pairs until an unique pair is found.
		++index;
		while (index < m_pairCount) 
		{
			const b3Pair* secondaryPair = m_pairs + index;
			if (secondaryPair->proxy1 != primaryPair->proxy1 || secondaryPair->proxy2 != primaryPair->proxy2) 
			{
				break;
			}
			++index;
		}
	}
}

inline void b3BroadPhase::Draw(b3Draw* draw) const
{
	m_tree.Draw(draw);
}

#endif
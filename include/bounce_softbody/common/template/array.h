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

#ifndef B3_ARRAY_POD_H
#define B3_ARRAY_POD_H

#include <bounce_softbody/common/settings.h>

// An array for bytes (POD).
template <typename T>
class b3Array
{
public:
	const T& operator[](uint32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_array[i];
	}

	T& operator[](uint32 i)
	{
		B3_ASSERT(i < m_count);
		return m_array[i];
	}

	const T* Get(uint32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_array + i;
	}

	T* Get(uint32 i)
	{
		B3_ASSERT(i < m_count);
		return m_array + i;
	}

	const T* Begin() const
	{
		return m_array;
	}

	T* Begin()
	{
		return m_array;
	}

	void PushBack(const T& ele)
	{
		if (m_count == m_capacity)
		{
			T* old = m_array;
			m_capacity *= 2;
			m_array = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_array, old, m_count * sizeof(T));
			if (old != m_memory)
			{
				b3Free(old);
			}
		}
		B3_ASSERT(m_count < m_capacity);
		m_array[m_count] = ele;
		++m_count;
	}

	void PopBack()
	{
		B3_ASSERT(m_count > 0);
		--m_count;
	}

	const T& Back() const
	{
		B3_ASSERT(m_count > 0);
		return m_array[m_count - 1];
	}

	T& Back()
	{
		B3_ASSERT(m_count > 0);
		return m_array[m_count - 1];
	}

	uint32 Capacity() const
	{
		return m_capacity;
	}

	uint32 Count() const
	{
		return m_count;
	}

	bool IsEmpty() const
	{
		return m_count == 0;
	}

	void Reserve(uint32 size)
	{
		if (m_capacity < size)
		{
			T* old = m_array;
			m_capacity = 2 * size;
			m_array = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_array, old, m_count * sizeof(T));
			if (old != m_memory)
			{
				b3Free(old);
			}
		}

		B3_ASSERT(m_capacity >= size);
	}

	void Resize(uint32 size)
	{
		Reserve(size);
		m_count = size;
	}

	void Copy(const b3Array<T>& other)
	{
		if (m_array == other.m_array)
		{
			return;
		}

		// Ensure sufficient capacity for copy.
		if (m_capacity < other.m_count)
		{
			if (m_array != m_memory)
			{
				b3Free(m_array);
			}
			m_capacity = other.m_capacity;
			m_array = (T*)b3Alloc(m_capacity * sizeof(T));
		}
		
		// Copy.
		B3_ASSERT(m_capacity >= other.m_count);
		m_count = other.m_count;
		memcpy(m_array, other.m_array, other.m_count * sizeof(T));
	}

	void operator=(const b3Array<T>& other)
	{
		Copy(other);
	}
protected:
	b3Array(T* memory, uint32 N)
	{
		B3_ASSERT(N > 0);
		m_memory = memory;
		m_capacity = N;
		m_array = m_memory;
		m_count = 0;
	}

	b3Array(const b3Array<T>& other)
	{
		m_memory = nullptr;
		m_capacity = 0;
		m_array = nullptr;
		m_count = 0;

		Copy(other);
	}

	~b3Array()
	{
		if (m_array != m_memory)
		{
			b3Free(m_array);
			m_array = nullptr;
		}
	}

	T* m_array;
	T* m_memory;
	uint32 m_capacity;
	uint32 m_count;
};

template <typename T, uint32 N>
class b3StackArray : public b3Array<T>
{
public :
	b3StackArray<T, N>() : b3Array<T>(m_stack, N) { }
	b3StackArray<T, N>(const b3StackArray<T, N>& other) : b3Array<T>(other) { }
	b3StackArray<T, N>(const b3Array<T>& other) : b3Array<T>(other) { }

	void operator=(const b3StackArray<T, N>& other)
	{
		b3Array<T>::Copy((const b3Array<T>&)other);
	}

	void operator=(const b3Array<T>& other)
	{
		b3Array<T>::Copy(other);
	}
protected:
	T m_stack[N];
};

#endif

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

#ifndef B3_SPARSE_MAT_33_H
#define B3_SPARSE_MAT_33_H

#include <bounce_softbody/common/math/mat33.h>
#include <bounce_softbody/sparse/diag_mat33.h>
#include <bounce_softbody/sparse/dense_vec3.h>

// Sparse matrix row element.
struct b3RowEntry
{
	uint32 column;
	b3Mat33 value;
	b3RowEntry* prev;
	b3RowEntry* next;
};

// Doubly linked list of row entries.
struct b3RowEntryList
{
	b3RowEntryList() { }

	void Insert(b3RowEntry* entry);

	void Remove(b3RowEntry* entry);

	b3RowEntry* Search(uint32 column);

	b3RowEntry* head;
	uint32 count;
};

inline void b3RowEntryList::Insert(b3RowEntry* entry)
{
	entry->prev = nullptr;
	entry->next = head;
	if (head)
	{
		head->prev = entry;
	}
	head = entry;
	++count;
}

inline void b3RowEntryList::Remove(b3RowEntry* entry)
{
	if (entry->prev)
	{
		entry->prev->next = entry->next;
	}
	if (entry->next)
	{
		entry->next->prev = entry->prev;
	}
	if (entry == head)
	{
		head = entry->next;
	}
	--count;
}

inline b3RowEntry* b3RowEntryList::Search(uint32 column)
{
	// Make O(log).
	for (b3RowEntry* e = head; e; e = e->next)
	{
		if (e->column == column)
		{
			return e;
		}
	}
	return nullptr;
}

// A sparse matrix.
// Each row is a list of non-zero elements in the row.
struct b3SparseMat33
{
	b3SparseMat33(uint32 m);

	b3SparseMat33(const b3SparseMat33& _m);

	~b3SparseMat33();

	b3SparseMat33& operator=(const b3SparseMat33& _m);

	void Copy(const b3SparseMat33& _m);

	void Destroy();

	void SetZero(uint32 i, uint32 j);

	void SetZeroRow(uint32 i);

	void SetZeroColumn(uint32 j);

	b3Mat33* Search(uint32 i, uint32 j);

	const b3Mat33* Search(uint32 i, uint32 j) const;

	b3Mat33& operator()(uint32 i, uint32 j);

	b3Mat33 operator()(uint32 i, uint32 j) const;

	void operator+=(const b3SparseMat33& m);

	void operator-=(const b3SparseMat33& m);

	void operator+=(const b3DiagMat33& m);

	void operator-=(const b3DiagMat33& m);

	uint32 GetElementCount() const { return 3 * rowCount * 3 * rowCount; }

	scalar& GetElement(uint32 i, uint32 j);

	scalar GetElement(uint32 i, uint32 j) const;

	void CreateMatrix(scalar* out) const;

	uint32 rowCount;
	b3RowEntryList* rows;
};

inline b3SparseMat33::b3SparseMat33(uint32 m)
{
	rowCount = m;
	rows = (b3RowEntryList*)b3Alloc(rowCount * sizeof(b3RowEntryList));
	for (uint32 i = 0; i < rowCount; ++i)
	{
		rows[i].head = nullptr;
		rows[i].count = 0;
	}
}

inline b3SparseMat33::b3SparseMat33(const b3SparseMat33& m)
{
	rowCount = m.rowCount;
	rows = (b3RowEntryList*)b3Alloc(rowCount * sizeof(b3RowEntryList));
	for (uint32 i = 0; i < rowCount; ++i)
	{
		rows[i].head = nullptr;
		rows[i].count = 0;
	}

	Copy(m);
}

inline b3SparseMat33::~b3SparseMat33()
{
	Destroy();
}

inline void b3SparseMat33::Destroy()
{
	for (uint32 i = 0; i < rowCount; ++i)
	{
		b3RowEntryList* list = rows + i;

		b3RowEntry* e = list->head;
		while (e)
		{
			b3RowEntry* e0 = e->next;
			b3Free(e);
			e = e0;
		}
	}
	b3Free(rows);
}

inline b3SparseMat33& b3SparseMat33::operator=(const b3SparseMat33& _m)
{
	if (_m.rows == rows)
	{
		return *this;
	}

	Destroy();

	rowCount = _m.rowCount;
	rows = (b3RowEntryList*)b3Alloc(rowCount * sizeof(b3RowEntryList));
	for (uint32 i = 0; i < rowCount; ++i)
	{
		rows[i].head = nullptr;
		rows[i].count = 0;
	}

	Copy(_m);

	return *this;
}

inline void b3SparseMat33::Copy(const b3SparseMat33& _m)
{
	B3_ASSERT(rowCount == _m.rowCount);

	for (uint32 i = 0; i < rowCount; ++i)
	{
		b3RowEntryList* list1 = _m.rows + i;
		b3RowEntryList* list2 = rows + i;

		B3_ASSERT(list2->head == nullptr);
		B3_ASSERT(list2->count == 0);
		for (b3RowEntry* e1 = list1->head; e1; e1 = e1->next)
		{
			b3RowEntry* e2 = (b3RowEntry*)b3Alloc(sizeof(b3RowEntry));
			e2->column = e1->column;
			e2->value = e1->value;

			list2->Insert(e2);
		}
	}
}

inline b3Mat33* b3SparseMat33::Search(uint32 i, uint32 j)
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);
	b3RowEntryList* list = rows + i;
	b3RowEntry* e = list->Search(j);
	if (e)
	{
		return &e->value;
	}
	return nullptr;
}

inline const b3Mat33* b3SparseMat33::Search(uint32 i, uint32 j) const
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);
	b3RowEntryList* list = rows + i;
	b3RowEntry* e = list->Search(j);
	if (e)
	{
		return &e->value;
	}
	return nullptr;
}

inline b3Mat33 b3SparseMat33::operator()(uint32 i, uint32 j) const
{
	const b3Mat33* v = ((b3SparseMat33*)this)->Search(i, j);
	if (v)
	{
		return *v;
	}
	return b3Mat33_zero;
}

inline b3Mat33& b3SparseMat33::operator()(uint32 i, uint32 j)
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	b3RowEntryList* list = rows + i;

	b3RowEntry* e = list->Search(j);
	if (e)
	{
		return e->value;
	}

	e = (b3RowEntry*)b3Alloc(sizeof(b3RowEntry));
	e->column = j;
	e->value.SetZero();

	list->Insert(e);

	return e->value;
}

inline void b3SparseMat33::SetZero(uint32 i, uint32 j)
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	b3RowEntryList* list = rows + i;

	b3RowEntry* e = list->Search(j);
	if (e)
	{
		list->Remove(e);
		b3Free(e);
	}
}

inline void b3SparseMat33::SetZeroRow(uint32 i)
{
	B3_ASSERT(i < rowCount);
	b3RowEntryList* list = rows + i;
	b3RowEntry* e = list->head;
	while (e)
	{
		b3RowEntry* boom = e;
		e = e->next;
		list->Remove(boom);
		b3Free(boom);
	}
}

inline void b3SparseMat33::SetZeroColumn(uint32 j)
{
	B3_ASSERT(j < rowCount);
	for (uint32 i = 0; i < rowCount; ++i)
	{
		b3RowEntryList* list = rows + i;
		b3RowEntry* e = list->Search(j);
		if (e)
		{
			list->Remove(e);
			b3Free(e);
		}
	}
}

inline void b3SparseMat33::operator+=(const b3SparseMat33& m)
{
	B3_ASSERT(rowCount == m.rowCount);

	for (uint32 i = 0; i < m.rowCount; ++i)
	{
		b3RowEntryList* list = m.rows + i;

		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			uint32 j = e->column;

			(*this)(i, j) += e->value;
		}
	}
}

inline void b3SparseMat33::operator-=(const b3SparseMat33& m)
{
	B3_ASSERT(rowCount == m.rowCount);

	for (uint32 i = 0; i < m.rowCount; ++i)
	{
		b3RowEntryList* list = m.rows + i;

		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			uint32 j = e->column;

			(*this)(i, j) -= e->value;
		}
	}
}

inline void b3SparseMat33::operator+=(const b3DiagMat33& m)
{
	B3_ASSERT(rowCount == m.n);
	for (uint32 i = 0; i < m.n; ++i)
	{
		(*this)(i, i) += m[i];
	}
}

inline void b3SparseMat33::operator-=(const b3DiagMat33& m)
{
	B3_ASSERT(rowCount == m.n);
	for (uint32 i = 0; i < m.n; ++i)
	{
		(*this)(i, i) -= m[i];
	}
}

inline scalar& b3SparseMat33::GetElement(uint32 i, uint32 j)
{
	B3_ASSERT(i < 3 * rowCount);
	B3_ASSERT(j < 3 * rowCount);

	uint32 i0 = i / 3;
	uint32 j0 = j / 3;

	b3Mat33& a = (*this)(i0, j0);

	uint32 ii = i - 3 * i0;
	uint32 jj = j - 3 * j0;

	return a(ii, jj);
}

inline scalar b3SparseMat33::GetElement(uint32 i, uint32 j) const
{
	B3_ASSERT(i < 3 * rowCount);
	B3_ASSERT(j < 3 * rowCount);

	uint32 i0 = i / 3;
	uint32 j0 = j / 3;

	b3Mat33 a = (*this)(i0, j0);

	uint32 ii = i - 3 * i0;
	uint32 jj = j - 3 * j0;

	return a(ii, jj);
}

inline void b3SparseMat33::CreateMatrix(scalar* out) const
{
	uint32 AM = 3 * rowCount;
	uint32 AN = AM;
	scalar* A = out;

	for (uint32 i = 0; i < AM * AN; ++i)
	{
		A[i] = scalar(0);
	}

	for (uint32 i = 0; i < rowCount; ++i)
	{
		b3RowEntryList* list = rows + i;

		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			uint32 j = e->column;
			b3Mat33 a = e->value;

			for (uint32 ii = 0; ii < 3; ++ii)
			{
				for (uint32 jj = 0; jj < 3; ++jj)
				{
					uint32 row = 3 * i + ii;
					uint32 col = 3 * j + jj;

					A[row + AM * col] = a(ii, jj);
				}
			}
		}
	}
}

inline void b3Add(b3SparseMat33& out, const b3SparseMat33& a, const b3SparseMat33& b)
{
	out = a;
	out += b;
}

inline void b3Sub(b3SparseMat33& out, const b3SparseMat33& a, const b3SparseMat33& b)
{
	out = a;
	out -= b;
}

inline void b3Add(b3SparseMat33& out, const b3SparseMat33& a, const b3DiagMat33& b)
{
	out = a;
	out += b;
}

inline void b3Sub(b3SparseMat33& out, const b3SparseMat33& a, const b3DiagMat33& b)
{
	out = a;
	out -= b;
}

inline void b3Add(b3SparseMat33& out, const b3DiagMat33& a, const b3SparseMat33& b)
{
	out = b;
	out += a;
}

inline void b3Sub(b3SparseMat33& out, const b3DiagMat33& a, const b3SparseMat33& b)
{
	B3_ASSERT(out.rowCount == a.n);

	for (uint32 i = 0; i < a.n; ++i)
	{
		out(i, i) = a[i];
	}

	out -= b;
}

inline void b3Mul(b3DenseVec3& out, const b3SparseMat33& A, const b3DenseVec3& v)
{
	B3_ASSERT(A.rowCount == out.n);

	out.SetZero();

	for (uint32 i = 0; i < A.rowCount; ++i)
	{
		b3RowEntryList* list = A.rows + i;

		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			uint32 j = e->column;
			b3Mat33 a = e->value;

			out[i] += a * v[j];
		}
	}
}

inline void b3Mul(b3SparseMat33& out, scalar s, const b3SparseMat33& B)
{
	B3_ASSERT(out.rowCount == B.rowCount);

	if (s == scalar(0))
	{
		return;
	}

	out = B;

	for (uint32 i = 0; i < out.rowCount; ++i)
	{
		b3RowEntryList* list = out.rows + i;
		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			e->value = s * e->value;
		}
	}
}

inline void b3Mul(b3SparseMat33& out, const b3DiagMat33& A, const b3SparseMat33& B)
{
	B3_ASSERT(A.n == B.rowCount);
	B3_ASSERT(out.rowCount == B.rowCount);

	out = B;
	
	// Pre multiplication:
	// [a11 0 ] [b11 b12] = [a11 * b11  a11 * b12]
	// [0  a22] [b21 b22]   [a22 * b21  a22 * b22]
	// cij = aii * bij
	for (uint32 i = 0; i < out.rowCount; ++i)
	{
		b3RowEntryList* list = out.rows + i;
		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			e->value = A[i] * e->value;
		}
	}
}

inline void b3Mul(b3SparseMat33& out, const b3SparseMat33& A, const b3DiagMat33& B)
{
	B3_ASSERT(A.rowCount == B.n);
	B3_ASSERT(out.rowCount == A.rowCount);

	out = A;

	// Post multiplication:
	// [a11 a12] * [b11 0 ] = [a11 * b11  a12 * b22]
	// [a21 a22]   [0  b22]   [a21 * b11  a22 * b22]
	// cij = aij * bjj
	for (uint32 i = 0; i < out.rowCount; ++i)
	{
		b3RowEntryList* list = out.rows + i;
		for (b3RowEntry* e = list->head; e; e = e->next)
		{
			uint32 j = e->column;
			e->value = e->value * B[j];
		}
	}
}

inline b3SparseMat33 operator+(const b3SparseMat33& A, const b3SparseMat33& B)
{
	b3SparseMat33 result(A.rowCount);
	b3Add(result, A, B);
	return result;
}

inline b3SparseMat33 operator-(const b3SparseMat33& A, const b3SparseMat33& B)
{
	b3SparseMat33 result(A.rowCount);
	b3Sub(result, A, B);
	return result;
}

inline b3SparseMat33 operator+(const b3SparseMat33& A, const b3DiagMat33& B)
{
	b3SparseMat33 result(A.rowCount);
	b3Add(result, A, B);
	return result;
}

inline b3SparseMat33 operator-(const b3SparseMat33& A, const b3DiagMat33& B)
{
	b3SparseMat33 result(A.rowCount);
	b3Sub(result, A, B);
	return result;
}

inline b3SparseMat33 operator+(const b3DiagMat33& A, const b3SparseMat33& B)
{
	b3SparseMat33 result(B.rowCount);
	b3Add(result, A, B);
	return result;
}

inline b3SparseMat33 operator-(const b3DiagMat33& A, const b3SparseMat33& B)
{
	b3SparseMat33 result(B.rowCount);
	b3Sub(result, A, B);
	return result;
}

inline b3SparseMat33 operator*(scalar A, const b3SparseMat33& B)
{
	b3SparseMat33 result(B.rowCount);
	b3Mul(result, A, B);
	return result;
}

inline b3DenseVec3 operator*(const b3SparseMat33& A, const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Mul(result, A, v);
	return result;
}

inline b3SparseMat33 operator*(const b3DiagMat33& A, const b3SparseMat33& B)
{
	b3SparseMat33 result(B.rowCount);
	b3Mul(result, A, B);
	return result;
}

inline b3SparseMat33 operator*(const b3SparseMat33& A, const b3DiagMat33& B)
{
	b3SparseMat33 result(A.rowCount);
	b3Mul(result, A, B);
	return result;
}

#endif
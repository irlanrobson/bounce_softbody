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

#ifndef B3_BODY_DRAGGER_H
#define B3_BODY_DRAGGER_H

#include <bounce/common/math/vec3.h>
#include <bounce/dynamics/particle.h>

struct b3Ray;
class b3Body;
class b3MouseForce;

// A body triangle dragger.
class BodyDragger
{
public:
	BodyDragger(b3Ray* ray, b3Body* body);

	void SetStaticDrag(bool bit);

	bool GetStaticDrag() const;

	void SetMouseStiffness(scalar k);

	scalar GetMouseStiffness();

	void SetMouseDamping(scalar k);

	scalar GetMouseDamping();
	
	bool IsDragging() const;

	bool StartDragging();

	void Drag();

	void StopDragging();

	b3Vec3 GetPointA() const;

	b3Vec3 GetPointB() const;
private:
	b3Ray* m_ray;
	scalar m_x;

	b3Body* m_body;
	
	bool m_isDragging;
	b3Particle* m_p1;
	b3Particle* m_p2;
	b3Particle* m_p3;
	scalar m_u, m_v;

	scalar m_km;
	scalar m_kd;
	b3Particle* m_particle;
	b3MouseForce* m_mf;

	bool m_staticDrag;
	b3ParticleType m_t1, m_t2, m_t3;
};

inline bool BodyDragger::GetStaticDrag() const
{
	return m_staticDrag;
}

inline void BodyDragger::SetMouseStiffness(scalar k)
{
	m_km = k;
}

inline scalar BodyDragger::GetMouseStiffness()
{
	return m_km;
}

inline void BodyDragger::SetMouseDamping(scalar k)
{
	m_kd = k;
}

inline scalar BodyDragger::GetMouseDamping()
{
	return m_kd;
}

inline bool BodyDragger::IsDragging() const
{
	return m_isDragging;
}

#endif
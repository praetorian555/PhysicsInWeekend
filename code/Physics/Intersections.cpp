//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact )
{
	const Vec3 dist = bodyB->m_position - bodyA->m_position;

	const auto sphereA = reinterpret_cast<const ShapeSphere*>(bodyA->m_shape);
	const auto sphereB = reinterpret_cast<const ShapeSphere*>(bodyB->m_shape);

	const float totalRadii = sphereA->m_radius + sphereB->m_radius;
	return dist.GetLengthSqr() <= (totalRadii * totalRadii);
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code

	return false;
}























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
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	const Vec3 dist = bodyB->m_position - bodyA->m_position;
	contact.normal = dist;
	contact.normal.Normalize();

	const auto sphereA = reinterpret_cast<const ShapeSphere*>(bodyA->m_shape);
	const auto sphereB = reinterpret_cast<const ShapeSphere*>(bodyB->m_shape);

	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

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























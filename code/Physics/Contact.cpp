//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact(contact_t& contact)
{
	Body& bodyA = *contact.bodyA;
	Body& bodyB = *contact.bodyB;

	const Vec3& n = contact.normal;
	const Vec3 vab = bodyA.m_linearVelocity - bodyB.m_linearVelocity;
	const float impulse = -2.0f * vab.Dot(n) / (bodyA.m_invMass + bodyB.m_invMass);
	const Vec3 impulseVector = n * impulse;

	bodyA.ApplyImpulseLinear(impulseVector);
	bodyB.ApplyImpulseLinear(impulseVector * -1.0f);

	const float tA = bodyA.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);
	const float tB = bodyB.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);

	const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA.m_position += ds * tA;
	bodyB.m_position -= ds * tB;
}
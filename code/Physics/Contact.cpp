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

	const float elasticity = bodyA.m_elasticity * bodyB.m_elasticity;

	const Mat3 inverseWorldInertiaA = bodyA.GetInverseInertiaTensorWorldSpace();
	const Mat3 inverseWorldInertiaB = bodyB.GetInverseInertiaTensorWorldSpace();

	const Vec3& n = contact.normal;

	const Vec3 ra = contact.ptOnA_WorldSpace - bodyA.GetCenterOfMassWorldSpace();
	const Vec3 rb = contact.ptOnB_WorldSpace - bodyB.GetCenterOfMassWorldSpace();

	const Vec3 angularJA = (inverseWorldInertiaA * ra.Cross(n)).Cross(ra);
	const Vec3 angularJB = (inverseWorldInertiaB * rb.Cross(n)).Cross(rb);
	const float angularFactor = (angularJA + angularJB).Dot(n);

	// Get the world space velocity of the motion and rotation
	const Vec3 velA = bodyA.m_linearVelocity + bodyA.m_angularVelocity.Cross(ra);
	const Vec3 velB = bodyB.m_linearVelocity + bodyB.m_angularVelocity.Cross(rb);

	// Calculate the collision impulse
	const Vec3 vab = velA - velB;
	const float impulse = (1.0f + elasticity) * vab.Dot(n) / (bodyA.m_invMass + bodyB.m_invMass + angularFactor);
	const Vec3 impulseVector = n * impulse;

	bodyA.ApplyImpulse(contact.ptOnA_WorldSpace, impulseVector * -1.0f);
	bodyB.ApplyImpulse(contact.ptOnB_WorldSpace, impulseVector * 1.0f);

	const float tA = bodyA.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);
	const float tB = bodyB.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);

	const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA.m_position += ds * tA;
	bodyB.m_position -= ds * tB;
}
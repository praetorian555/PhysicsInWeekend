//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact(const contact_t& contact)
{
	Body& bodyA = *contact.bodyA;
	Body& bodyB = *contact.bodyB;

	const float elasticity = bodyA.m_elasticity * bodyB.m_elasticity;
	const float friction = bodyA.m_friction * bodyB.m_friction;

	const Mat3 inverseWorldInertiaA = bodyA.GetInverseInertiaTensorWorldSpace();
	const Mat3 inverseWorldInertiaB = bodyB.GetInverseInertiaTensorWorldSpace();

	const Vec3& n = contact.normal;

	const Vec3 ra = contact.ptOnA_WorldSpace - bodyA.GetCenterOfMassWorldSpace();
	const Vec3 rb = contact.ptOnB_WorldSpace - bodyB.GetCenterOfMassWorldSpace();

	//
	// Calculate total impulse and apply it to the bodies
	//

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

	//
	// Apply impulse due to friction
	//

	const Vec3 velocityNormal = n * n.Dot(vab);
	const Vec3 velocityTangent = vab - velocityNormal;

	Vec3 relativeVelocityTangent = velocityTangent;
	relativeVelocityTangent.Normalize();

	const Vec3 inertiaA = (inverseWorldInertiaA * ra.Cross(relativeVelocityTangent)).Cross(ra);
	const Vec3 inertiaB = (inverseWorldInertiaB * rb.Cross(relativeVelocityTangent)).Cross(rb);
	const float inverseInertia = (inertiaA + inertiaB).Dot(relativeVelocityTangent);

	const float reducedMass = 1.0f / (bodyA.m_invMass + bodyB.m_invMass + inverseInertia);
	const Vec3 frictionImpulse = velocityTangent * reducedMass * friction;

	bodyA.ApplyImpulse(contact.ptOnA_WorldSpace, frictionImpulse * -1.0f);
	bodyB.ApplyImpulse(contact.ptOnB_WorldSpace, frictionImpulse);

	//
	// Resolve interpenetration
	//

	if (contact.timeOfImpact == 0.0f)
	{
		const float tA = bodyA.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);
		const float tB = bodyB.m_invMass / (bodyA.m_invMass + bodyB.m_invMass);

		const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
		bodyA.m_position += ds * tA;
		bodyB.m_position -= ds * tB;
	}
}

int CompareContacts(const contact_t& c1, const contact_t& c2)
{

	if (c1.timeOfImpact < c2.timeOfImpact)
	{
		return -1;
	}
	if (c1.timeOfImpact == c2.timeOfImpact)
	{
		return 0;
	}
	return 1;
}

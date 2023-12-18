//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
	m_position(0.0f),
	m_orientation(0.0f, 0.0f, 0.0f, 1.0f),
	m_linearVelocity(0, 0, 0),
	m_invMass(0.0f),
	m_shape(nullptr)
{}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPoint) const
{
	const Vec3 point = worldPoint - GetCenterOfMassWorldSpace();
	const Quat invOrient = m_orientation.Inverse();
	const Vec3 bodyPoint = invOrient.RotatePoint(point);
	return bodyPoint;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& bodyPoint) const
{
	const Vec3 worldPoint = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(bodyPoint);
	return worldPoint;
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	// m * v = p <- moment
	// m * dv = dp <- change of moment in unit of time or impulse
	// dv = dp * inv_m
	m_linearVelocity += impulse * m_invMass;
}

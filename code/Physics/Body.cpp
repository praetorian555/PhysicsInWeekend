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
	m_elasticity(1.0f),
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

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	const Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return inverseInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	const Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	const Mat3 orient = m_orientation.ToMat3();
	inverseInertiaTensor = orient * inverseInertiaTensor * orient.Transpose();
	return inverseInertiaTensor;
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	// m * v = p <- moment
	// m * dv = dp <- change of moment in unit of time or impulse
	// dv = dp * inv_m
	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}

	// L = I w = r x p
	// dL = I dw = r x J
	// dw = I^-1 * (r x J)
	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	constexpr float kMaxAngularSpeed = 30.0f; 
	constexpr float kMaxAngularSpeedSq = kMaxAngularSpeed * kMaxAngularSpeed;
	if (m_angularVelocity.GetLengthSqr() > kMaxAngularSpeedSq)
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= kMaxAngularSpeed;
	}
}

void Body::ApplyImpulse(const Vec3& point, const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}

	ApplyImpulseLinear(impulse);

	const Vec3 cm = GetCenterOfMassWorldSpace();
	const Vec3 r = point - cm;
	const Vec3 angularImpulse = r.Cross(impulse);
	ApplyImpulseAngular(angularImpulse);
}

void Body::Update(float dt_sec)
{
	m_position += m_linearVelocity * dt_sec;

	const Vec3 cm = GetCenterOfMassWorldSpace();
	const Vec3 cmToPosition = m_position - cm;

	// Total torque is equal to external torques + internal torque (precession)
	// T = T_external + omega x I * omega
	// Now T_external is 0 since it was already applied in contact resolution function
	// T = Ia = w x I * w
	// a = I^-1 (w x I * w)
	const Mat3 orientation = m_orientation.ToMat3();
	const Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
	const Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor * m_angularVelocity));
	m_angularVelocity += alpha * dt_sec;

	const Vec3 dAngle = m_angularVelocity * dt_sec;
	const Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	m_position = cm + dq.RotatePoint(cmToPosition);
}

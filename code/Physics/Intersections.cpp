//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

bool RaySphere(const Vec3& rayStart, const Vec3& rayDirection, const Vec3& sphereCenter, const float sphereRadius, float& t1, float& t2)
{
	const Vec3 m = sphereCenter - rayStart;
	const float a = rayDirection.Dot(rayDirection);
	const float b = m.Dot(rayDirection);
	const float c = m.Dot(m) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float inva = 1.0f / a;

	if (delta < 0)
	{
		// No real solutions exist
		return false;
	}

	const float deltaRoot = sqrtf(delta);
	t1 = inva * (b - deltaRoot);
	t2 = inva * (b + deltaRoot);

	return true;
}

bool SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& positionA, const Vec3& positionB, const Vec3& velocityA, const Vec3& velocityB,
	const float dt, Vec3& ptOnA, Vec3& ptOnB, float& toi)
{
	const Vec3 relativeVelocity = velocityA - velocityB;

	const Vec3 startPtA = positionA;
	const Vec3 endPtA = positionA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;

	float t1 = 0;
	float t2 = 0;
	if (rayDir.GetLengthSqr() < 0.001f * 0.001f)
	{
		// Ray is too short, just check if we are intersecting
		Vec3 ab = positionB - positionA;
		float radius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius)
		{
			return false;
		}
	}
	else if (!RaySphere(positionA, rayDir, positionB, shapeA->m_radius + shapeB->m_radius, t1, t2))
	{
		return false;
	}

	// Change from the range [0, 1] to the range [0, dt]
	t1 *= dt;
	t2 *= dt;

	if (t2 < 0.0f)
	{
		// If both collisions are in the past, then there is no collisions this frame
		return false;
	}

	// Get the earliest positive time of the impact 
	toi = (t1 < 0.0f) ? 0.0f : t1;

	if (toi > dt)
	{
		// If the earliest collision is too far into the future, then there is no collision this frame
		return false;
	}

	Vec3 newPositionA = positionA + velocityA * toi;
	Vec3 newPositionB = positionB + velocityB * toi;
	Vec3 normal = newPositionB - newPositionA;
	normal.Normalize();

	ptOnA = newPositionA + normal * shapeA->m_radius;
	ptOnB = newPositionB - normal * shapeB->m_radius;
	return true;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact)
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
bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact)
{
	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE)
	{
		contact.bodyA = bodyA;
		contact.bodyB = bodyB;

		const ShapeSphere* sphereA = reinterpret_cast<const ShapeSphere*>(bodyA->m_shape);
		const ShapeSphere* sphereB = reinterpret_cast<const ShapeSphere*>(bodyB->m_shape);

		const Vec3 positionA = bodyA->m_position;
		const Vec3 positionB = bodyB->m_position;

		const Vec3 velocityA = bodyA->m_linearVelocity;
		const Vec3 velocityB = bodyB->m_linearVelocity;

		if (SphereSphereDynamic(sphereA, sphereB, positionA, positionB, velocityA, velocityB, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact))
		{
			// Step bodies forward to get local space collision points
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);

			// Convert world space contact points to local space
			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.normal = bodyB->m_position - bodyA->m_position;
			contact.normal.Normalize();

			// Unwind time step
			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);

			// Calculate the separation distance
			Vec3 ab = bodyB->m_position - bodyA->m_position;
			contact.separationDistance = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			return true;
		}
	}


	return false;
}























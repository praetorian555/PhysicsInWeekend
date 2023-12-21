//
//  Scene.cpp
//
#include <vector>
#include <algorithm>

#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for (int i = 0; i < m_bodies.size(); i++)
	{
		delete m_bodies[i].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset()
{
	for (int i = 0; i < m_bodies.size(); i++)
	{
		delete m_bodies[i].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
	Body body;
	body.m_position = Vec3(-3, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(1000, 0, 0);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	m_bodies.push_back(body);

	body = Body{};
	body.m_position = Vec3(0, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	m_bodies.push_back(body);

	body = Body{};
	body.m_position = Vec3(0, 0, -1000);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 0.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_shape = new ShapeSphere(1000.0f);
	m_bodies.push_back(body);
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
	// Apply gravitational impulse
	for (int i = 0; i < m_bodies.size(); i++)
	{
		Body& body = m_bodies[i];
		const float mass = body.m_invMass > 0.0f ? 1.0f / body.m_invMass : 0.0f;
		const Vec3 gravityImpulse = Vec3(0, 0, -50.0f) * mass * dt_sec;
		body.ApplyImpulseLinear(gravityImpulse);
	}

	const int maxContacts = m_bodies.size() * m_bodies.size();
	std::vector<contact_t> contacts;
	contacts.reserve(maxContacts);
	
	// Collect all contacts
	for (int i = 0; i < m_bodies.size(); i++)
	{
		for (int j = i + 1; j < m_bodies.size(); j++)
		{
			Body& bodyA = m_bodies[i];
			Body& bodyB = m_bodies[j];

			if (bodyA.m_invMass == 0.0f && bodyB.m_invMass == 0.0f)
			{
				continue;
			}

			contact_t contact;
			if (Intersect(&bodyA, &bodyB, dt_sec, contact))
			{
				contacts.push_back(contact);
			}
		}
	}

	// Sort all contacts based on time of impact from earlieast to latest
	std::sort(contacts.begin(), contacts.end(), [](const contact_t& a, const contact_t& b) {
		return CompareContacts(a, b) < 0;
	});

	// Move the system from the current state to the earliest time of impact and so on until all of the
	// contacts are resolved
	float accumulatedTime = 0.0f;
	for (const contact_t& c : contacts)
	{
		const float dt = c.timeOfImpact - accumulatedTime;

		if (c.bodyA->m_invMass == 0.0f && c.bodyB->m_invMass == 0.0f)
		{
			continue;
		}

		// Position update until time of impact
		for (Body& body : m_bodies)
		{
			body.Update(dt);
		}

		ResolveContact(c);
		accumulatedTime += dt;
	}

	// If there is any time left in the frame, update the system to the end of the frame
	const float remainingTime = dt_sec - accumulatedTime;
	if (remainingTime > 0.0f)
	{
		for (Body& body : m_bodies)
		{
			body.Update(remainingTime);
		}
	}
}

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

void AddStandardSandBox(std::vector<Body>& bodies)
{
	Body body;

	body.m_position = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
	bodies.push_back(body);

	body.m_position = Vec3(50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	bodies.push_back(body);

	body.m_position = Vec3(-50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	bodies.push_back(body);

	body.m_position = Vec3(0, 25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	bodies.push_back(body);

	body.m_position = Vec3(0, -25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	bodies.push_back(body);
}

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

#if 0
	// Dynamic bodies
	for (int x = 0; x < 6; x++)
	{
		for (int y = 0; y < 6; y++)
		{
			const float radius = 0.5f;
			const float xx = float(x - 1) * radius * 1.5f;
			const float yy = float(y - 1) * radius * 1.5f;
			body.m_position = Vec3(xx, yy, 10.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity = Vec3(0, 0, 0);
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	// Static floor
	for (int x = 0; x < 3; x++)
	{
		for (int y = 0; y < 3; y++)
		{
			const float radius = 80.0f;
			const float xx = float(x - 1) * radius * 0.25f;
			const float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity = Vec3(0, 0, 0);
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}
#else

	body.m_position = Vec3(0, 0, 10);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
	m_bodies.push_back(body);

	AddStandardSandBox(m_bodies);

#endif
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

	//
	// Broadphase
	//
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	//
	// Narrowphase
	//
	int numContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = (contact_t*)_malloca(sizeof(contact_t) * maxContacts);
	assert(contacts != nullptr);

	// Collect all contacts
	for (int i = 0; i < collisionPairs.size(); i++)
	{
		collisionPair_t& pair = collisionPairs[i];
		Body& bodyA = m_bodies[pair.a];
		Body& bodyB = m_bodies[pair.b];

		if (bodyA.m_invMass == 0.0f && bodyB.m_invMass == 0.0f)
		{
			continue;
		}

		contact_t contact;
		if (Intersect(&bodyA, &bodyB, dt_sec, contact))
		{
			contacts[numContacts++] = contact;
		}
	}

	// Sort all contacts based on time of impact from earlieast to latest
	if (numContacts > 1)
	{
		std::qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	// Move the system from the current state to the earliest time of impact and so on until all of the
	// contacts are resolved
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		const contact_t& c = contacts[i];

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

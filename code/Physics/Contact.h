//
//	Contact.h
//
#pragma once
#include "Body.h"


struct contact_t {
	Vec3 ptOnA_WorldSpace;
	Vec3 ptOnB_WorldSpace;
	Vec3 ptOnA_LocalSpace;
	Vec3 ptOnB_LocalSpace;

	Vec3 normal;	// In World Space coordinates
	float separationDistance = -1000;	// positive when non-penetrating, negative when penetrating
	float timeOfImpact = -1000.0f;

	Body* bodyA = nullptr;
	Body* bodyB = nullptr;
};

void ResolveContact(const contact_t& contact);

int CompareContacts(const contact_t& c1, const contact_t& c2);
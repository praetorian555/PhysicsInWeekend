//
//  Broadphase.cpp
//
#include "Broadphase.h"

struct pseudoBody_t
{
	int id;
	float value;
	bool isMin;
};

int CompareSAP(const void* a, const void* b)
{
	const pseudoBody_t* bodyA = reinterpret_cast<const pseudoBody_t*>(a);
	const pseudoBody_t* bodyB = reinterpret_cast<const pseudoBody_t*>(b);
	return bodyA->value < bodyB->value ? -1 : 1;
}

void SortBodiesBounds(const Body* bodies, const int num, pseudoBody_t* sortedArray, const float dt_sec)
{
	Vec3 axis = Vec3(1, 1, 1);
	axis.Normalize();

	for (int i = 0; i < num; i++)
	{
		const Body& body = bodies[i];
		Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

		// Expand the bounds by the velocity
		bounds.Expand(bounds.mins + body.m_linearVelocity * dt_sec);
		bounds.Expand(bounds.maxs + body.m_linearVelocity * dt_sec);

		const float epsilon = 0.01f;
		bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
		bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

		const int array_idx = 2 * i;
		sortedArray[array_idx].id = i;
		sortedArray[array_idx].value = axis.Dot(bounds.mins);
		sortedArray[array_idx].isMin = true;

		sortedArray[array_idx + 1].id = i;
		sortedArray[array_idx + 1].value = axis.Dot(bounds.maxs);
		sortedArray[array_idx + 1].isMin = false;
	}

	std::qsort(sortedArray, 2 * num, sizeof(pseudoBody_t), CompareSAP);
}

void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const pseudoBody_t* sortedBodies, const int num)
{
	collisionPairs.clear();

	for (int i = 0; i < 2 * num; i++)
	{
		const pseudoBody_t& a = sortedBodies[i];
		if (!a.isMin)
		{
			continue;
		}

		collisionPair_t pair;
		pair.a = a.id;

		for (int j = i + 1; j < 2 * num; j++)
		{
			const pseudoBody_t& b = sortedBodies[j];
			// If we hit the end of the a element, we are done creating pairs for it
			if (b.id == a.id)
			{
				break;
			}

			if (!b.isMin)
			{
				continue;
			}

			pair.b = b.id;
			collisionPairs.push_back(pair);
		}
	}
}

void SweepAndPrune1D(const Body* bodies, const int num, std::vector<collisionPair_t>& finalPairs, const float dt_sec)
{
	pseudoBody_t* sortedBodies = (pseudoBody_t*)_malloca(2 * num * sizeof(pseudoBody_t));
	SortBodiesBounds(bodies, num, sortedBodies, dt_sec);
	BuildPairs(finalPairs, sortedBodies, num);
}

/*
====================================================
BroadPhase
====================================================
*/
void BroadPhase(const Body* bodies, const int num, std::vector< collisionPair_t >& finalPairs, const float dt_sec)
{
	finalPairs.clear();
	SweepAndPrune1D(bodies, num, finalPairs, dt_sec);
}
//
//  Shapes.cpp
//
#include "ShapeBox.h"

/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
====================================================
*/
void ShapeBox::Build( const Vec3 * pts, const int num ) {
	for (int i = 0; i < num; i++)
	{
		m_bounds.Expand(pts[i]);
	}

	m_points.clear();
	m_points.emplace_back(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
	m_points.emplace_back(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);
	m_points.emplace_back(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	m_points.emplace_back(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);

	m_points.emplace_back(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
	m_points.emplace_back(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);
	m_points.emplace_back(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	m_points.emplace_back(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);

	m_centerOfMass = (m_bounds.mins + m_bounds.maxs) * 0.5f;
}

/*
====================================================
ShapeBox::Support
====================================================
*/
Vec3 ShapeBox::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	// dir should be normalized
	Vec3 support_pt = orient.RotatePoint(m_points[0]) + pos;
	float max_dist = support_pt.Dot(dir);
	for (int i = 1; i < m_points.size(); i++)
	{
		Vec3 candidate = orient.RotatePoint(m_points[i]) + pos;
		float candidate_dist = candidate.Dot(dir);
		if (candidate_dist > max_dist)
		{
			support_pt = m_points[i];
			max_dist = candidate_dist;
		}
	}
	return support_pt + dir * bias;
}

/*
====================================================
ShapeBox::InertiaTensor
====================================================
*/
Mat3 ShapeBox::InertiaTensor() const {
	const float dx = m_bounds.maxs.x - m_bounds.mins.x;
	const float dy = m_bounds.maxs.y - m_bounds.mins.y;
	const float dz = m_bounds.maxs.z - m_bounds.mins.z;

	Mat3 tensor;
	tensor.rows[0][0] = (dy * dy + dz * dz) / 12.0f;
	tensor.rows[1][1] = (dx * dx + dz * dz) / 12.0f;
	tensor.rows[2][2] = (dx * dx + dy * dy) / 12.0f;

	const Vec3 center_of_mass = (m_bounds.mins + m_bounds.maxs) * 0.5f;
	const Vec3 r = Vec3(0, 0, 0) - center_of_mass;
	const float r2 = r.GetLengthSqr();

	Mat3 pat_tensor;
	pat_tensor.rows[0] = Vec3(r2 - r.x * r.x, r.x * r.y, r.x * r.z);
	pat_tensor.rows[1] = Vec3(r.y * r.x, r2 - r.y * r.y, r.y * r.z);
	pat_tensor.rows[2] = Vec3(r.z * r.x, r.z * r.y, r2 - r.z * r.z);

	tensor += pat_tensor;
	return tensor;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Vec3 corners[8];
	corners[0] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
	corners[1] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);
	corners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[3] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[4] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
	corners[5] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);
	corners[6] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[7] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);

	Bounds bounds;
	for (int i = 0; i < 8; i++)
	{
		corners[i] = orient.RotatePoint(corners[i]) + pos;
		bounds.Expand(corners[i]);
	}
	return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
====================================================
*/
float ShapeBox::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float max_speed = 0.0f;
	for (int i = 0; i < m_points.size(); i++)
	{
		const Vec3 r = m_points[i] - m_centerOfMass;
		const Vec3 linearVelocity = angularVelocity.Cross(r);
		const float speed = dir.Dot(linearVelocity);
		if (speed > max_speed)
		{
			max_speed = speed;
		}
	}
	return max_speed;
}
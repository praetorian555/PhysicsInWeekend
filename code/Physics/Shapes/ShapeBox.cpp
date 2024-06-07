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
	Vec3 supportPt;
	
	// TODO: Add code

	return supportPt;
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
	Bounds bounds;
	
	// TODO: Add code

	return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
====================================================
*/
float ShapeBox::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f;
	
	// TODO: Add code

	return maxSpeed;
}
//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"

/*
========================================================================================================

ShapeConvex

========================================================================================================
*/

int FindPointFurthestInDir(const Vec3* points, int num, const Vec3& dir)
{
	int max_idx = 0;
	float max_dist = dir.Dot(points[0]);
	for (int i = 1; i < num; i++)
	{
		const float dist = dir.Dot(points[i]);
		if (dist > max_dist)
		{
			max_dist = dist;
			max_idx = i;
		}
	}
	return max_idx;
}

float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& point)
{
	Vec3 ab = b - a;
	ab.Normalize();

	const Vec3 ray = point - a;
	const Vec3 projection = ab * ray.Dot(ab);
	const Vec3 perpendicular = ray - projection;
	return perpendicular.GetMagnitude();
}

Vec3 FindPointFurthestFromLine(const Vec3* points, int num, const Vec3& a, const Vec3& b)
{
	int max_idx = 0;
	float max_dist = DistanceFromLine(a, b, points[0]);
	for (int i = 1; i < num; i++)
	{
		const float dist = DistanceFromLine(a, b, points[i]);
		if (dist > max_dist)
		{
			max_dist = dist;
			max_idx = i;
		}
	}
	return points[max_idx];
}

float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& point)
{
	const Vec3 ab = b - a;
	const Vec3 ac = c - a;
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	const Vec3 ray = point - a;
	const float dist = ray.Dot(normal);
	return dist;
}

Vec3 FindPointFurthestFromTriangle(const Vec3* points, int num, const Vec3& a, const Vec3& b, const Vec3& c)
{
	float max_dist = 0;
	int max_idx = 0;
	max_dist = DistanceFromTriangle(a, b, c, points[0]);
	for (int i = 1; i < num; i++)
	{
		const float dist = DistanceFromTriangle(a, b, c, points[i]);
		if (dist > max_dist)
		{
			max_dist = dist;
			max_idx = i;
		}
	}
	return points[max_idx];
}

void BuildTetrahedron(const Vec3* vertices, int num, std::vector<Vec3>& out_hull_points, std::vector<tri_t>& out_hull_triangles)
{
	out_hull_points.clear();
	out_hull_triangles.clear();

	Vec3 points[4];

	int idx = FindPointFurthestInDir(vertices, num, Vec3(1, 0, 0));
	points[0] = vertices[idx];
	idx = FindPointFurthestInDir(vertices, num, points[0] * (-1.0f));
	points[1] = vertices[idx];
	points[2] = FindPointFurthestFromLine(vertices, num, points[0], points[1]);
	points[3] = FindPointFurthestFromTriangle(vertices, num, points[0], points[1], points[2]);

	// Make sure that all tetrahedron triangles are in CCW order, so that the normals will be directed outward.
	const float dist = DistanceFromTriangle(points[0], points[1], points[2], points[3]);
	if (dist > 0.0f)
	{
		std::swap(points[0], points[1]);
	}

	// Build tetrahedron
	out_hull_points.push_back(points[0]);
	out_hull_points.push_back(points[1]);
	out_hull_points.push_back(points[2]);
	out_hull_points.push_back(points[3]);

	tri_t tri;
	tri.a = 0;
	tri.b = 1;
	tri.c = 2;
	out_hull_triangles.push_back(tri);
	tri.a = 0;
	tri.b = 2;
	tri.c = 3;
	out_hull_triangles.push_back(tri);
	tri.a = 2;
	tri.b = 1;
	tri.c = 3;
	out_hull_triangles.push_back(tri);
	tri.a = 1;
	tri.b = 0;
	tri.c = 3;
	out_hull_triangles.push_back(tri);
}

void RemoveInternalPoints(const std::vector<Vec3>& hull_points, const std::vector<tri_t>& hull_triangles, std::vector<Vec3>& check_points)
{
	// Remove all points that are inside of the convex hull
	for (int i = 0; i < check_points.size(); i++)
	{
		const Vec3& point = check_points[i];

		bool is_external = false;
		for (int j = 0; j < hull_triangles.size(); j++)
		{
			const tri_t& triangle = hull_triangles[j];
			const Vec3& a = hull_points[triangle.a];
			const Vec3& b = hull_points[triangle.b];
			const Vec3& c = hull_points[triangle.c];

			const float dist = DistanceFromTriangle(a, b, c, point);
			if (dist > 0.0f)
			{
				is_external = true;
				break;
			}

			if (!is_external)
			{
				check_points.erase(check_points.begin() + i);
				i--;
			}
		}
	}

	// Also remove all points that are too close
	constexpr float k_too_close_threshold = 0.01f; // 1 cm
	for (int i = 0; i < check_points.size(); i++)
	{
		const Vec3& point = check_points[i];
		bool is_too_close = false;
		for (int j = 0; j < hull_points.size(); j++)
		{
			const Vec3& hull_point = hull_points[j];
			const Vec3 ray = hull_point - point;
			if (ray.GetLengthSqr() < k_too_close_threshold * k_too_close_threshold)
			{
				is_too_close = true;
				break;
			}
		}

		if (is_too_close)
		{
			check_points.erase(check_points.begin() + i);
			i--;
		}
	}
}

void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris ) {
	// TODO: Add code
}

/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build( const Vec3 * pts, const int num ) {
	// TODO: Add code
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;
	
	// TODO: Add code

	return supportPt;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds( const Vec3 & pos, const Quat & orient ) const {
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
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
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
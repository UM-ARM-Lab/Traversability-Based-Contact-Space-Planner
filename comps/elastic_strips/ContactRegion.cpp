
#include "stdafx.h"

ContactManipGroup::ContactManipGroup(int i, string name, Transform cct, std::vector<size_t> wp)
{
	group_index = i;
	manip_name = name;
	contact_consistent_transform = cct;
	waypoints = wp;
}

ContactRegion::ContactRegion(Vector p, Vector n, float r)
{
	position = p;
	normal = n;
	radius = r;

	TransformMatrix contact_region_frame_matrix;
	contact_region_frame_matrix.trans = position;

	Vector x_axis;
	Vector y_axis;
	Vector z_axis = -normal;

	if(z_axis.x != 0 or z_axis.y != 0)
	{
		x_axis = Vector(z_axis.y,-z_axis.x,0);
	}
	else
	{
		x_axis = Vector(1,0,0);
	}

	y_axis = z_axis.cross(x_axis);

	contact_region_frame_matrix.m[0] = x_axis.x; contact_region_frame_matrix.m[1] = y_axis.x; contact_region_frame_matrix.m[2] = z_axis.x;
	contact_region_frame_matrix.m[4] = x_axis.y; contact_region_frame_matrix.m[5] = y_axis.y; contact_region_frame_matrix.m[6] = z_axis.y;
	contact_region_frame_matrix.m[8] = x_axis.z; contact_region_frame_matrix.m[9] = y_axis.z; contact_region_frame_matrix.m[10] = z_axis.z;

	contact_region_frame = Transform(contact_region_frame_matrix);
	
}

float ContactRegion::TranslationDistToContactRegion(Transform contact_transform)
{
	Vector center_translation = contact_transform.trans - position;

	float z_dist = fabs(center_translation.dot(normal));

	float xy_dist = max(sqrt(center_translation.lengthsqr3() - pow(z_dist,2)) - radius, 0.0);

	float translation_dist = sqrt(pow(xy_dist,2) + pow(z_dist,2));

	return translation_dist;
}

float ContactRegion::OrientationDistToContactRegion(string contact_manip, Transform contact_transform)
{
	TransformMatrix contact_transform_matrix = TransformMatrix(contact_transform);
	Vector contact_normal;

	// if(strcmp(contact_manip.c_str(),'l_arm') == 0 || strcmp(contact_manip.c_str(),'r_arm') == 0)
	// {
	// 	contact_normal.x = contact_transform_matrix.m[0];
	// 	contact_normal.y = contact_transform_matrix.m[4];
	// 	contact_normal.z = contact_transform_matrix.m[8];
	// }
	// else if(strcmp(contact_manip.c_str(),'l_leg') == 0 || strcmp(contact_manip.c_str(),'r_leg') == 0)
	// {
	// 	contact_normal.x = -contact_transform_matrix.m[2];
	// 	contact_normal.y = -contact_transform_matrix.m[6];
	// 	contact_normal.z = -contact_transform_matrix.m[10];
	// }

	contact_normal.x = -contact_transform_matrix.m[2];
	contact_normal.y = -contact_transform_matrix.m[6];
	contact_normal.z = -contact_transform_matrix.m[10];

	float orientation_dist = 1 - fabs(contact_normal.dot(normal));

	return orientation_dist;
}

float ContactRegion::DistToContactRegion(string contact_manip, Transform contact_transform)
{
	float translation_dist = TranslationDistToContactRegion(contact_transform);
	float orientation_dist = OrientationDistToContactRegion(contact_manip,contact_transform);

	float k = 0.5;

	return (translation_dist + k * orientation_dist);
}

float ContactRegion::DistToContactRegion(string contact_manip, Transform contact_transform, Vector project_position)
{
	float translation_dist = sqrt((contact_transform.trans-project_position).lengthsqr3());
	float orientation_dist = OrientationDistToContactRegion(contact_manip,contact_transform);

	float k = 0.5;

	return (translation_dist + k * orientation_dist);
}
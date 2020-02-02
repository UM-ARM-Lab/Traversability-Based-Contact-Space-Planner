#ifndef  CONTACTREGION_H
#define  CONTACTREGION_H

class ContactManipGroup
{
	public:
		ContactManipGroup(int i, string name, Transform cct, std::vector<size_t> wp);
		int group_index;
		string manip_name;
		Transform contact_consistent_transform;
		std::vector<size_t> waypoints;
		std::set<int> avoid_contact_manip_group;
};

class ContactRegion
{
	public:
		ContactRegion(){};
		ContactRegion(Vector p, Vector n, float r);
		float DistToContactRegion(string contact_manip, Transform contact_transform);
		float DistToContactRegion(string contact_manip, Transform contact_transform, Vector project_position);
		float TranslationDistToContactRegion(Transform contact_transform);
		float OrientationDistToContactRegion(string contact_manip, Transform contact_transform);
		Transform contact_region_frame;
		Vector position;
		Vector normal;
		float radius;
	private:

};

#endif
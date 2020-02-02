#ifndef STRUCTURE_HPP
#define STRUCTURE_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class Structure
{

public:
	Structure(OpenRAVE::KinBodyPtr _kinbody, int _id) : kinbody(_kinbody), id(_id) {};
	OpenRAVE::KinBodyPtr getKinbody() const { return kinbody; }
	inline int getId(){return id;}

protected:
	// static int num_structures;
	OpenRAVE::KinBodyPtr kinbody;
	int id;

	// void setName() { kinbody->SetName(std::to_string(id)); }

};

#endif
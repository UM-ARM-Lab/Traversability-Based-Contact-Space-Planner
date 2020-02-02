#ifndef TRIMESHSURFACE_HPP
#define TRIMESHSURFACE_HPP

// #include "Utilities.hpp"
#include "Boundary.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

// TODO: verify the projection functions work as expected.

class TrimeshSurface : public Structure
{
public:
	TrimeshSurface(OpenRAVE::KinBodyPtr _kinbody, Eigen::Vector4f _plane_parameters,
				   std::vector< std::pair<int, int> > _edges, std::vector<Translation3D> _vertices, TrimeshType _type, int _id);
	TrimeshSurface(OpenRAVE::EnvironmentBasePtr _env, std::string _kinbody_name, Eigen::Vector4f _plane_parameters,
			       std::vector< std::pair<int, int> > _edges, std::vector<Translation3D> _vertices, TrimeshType _type, int _id);

	// OpenRAVE::TriMesh get_openrave_trimesh() const;

	inline float getMinProjX() const { return min_proj_x_; }
	inline float getMaxProjX() const { return max_proj_x_; }
	inline float getMinProjY() const { return min_proj_y_; }
	inline float getMaxProjY() const { return max_proj_y_; }

	void transform_data(TransformationMatrix transform);
	inline Translation3D getNormal() const {return Translation3D(nx_, ny_, nz_);}
	inline Translation3D getCenter() const {return Translation3D(xo_, yo_, zo_);}
	inline TransformationMatrix getTransform() const {return transformation_matrix_;}
	inline TransformationMatrix getInverseTransform() const {return inverse_transformation_matrix_;}
	inline TrimeshType getType() const {return type_;}
	inline float getCircumRadius() const {return circum_radius_;}

	// returns 2D point projected in plane frame. This assumes the "ray" is the surface normal.
	Translation2D projectionPlaneFrame(const Translation3D& start_point) const;
	Translation2D projectionPlaneFrame(const Translation3D& start_point, const Translation3D& ray) const;

	// returns a 3D point in global frame from a 2D point on the surface
	Translation3D getGlobalPosition(const Translation2D& point) const;

	
    // returns 3D point projected in plane frame. Ray is a 3D unit vector.
	Translation3D projectionGlobalFrame(const Translation3D& start_point, const Translation3D& ray) const;
	bool insidePolygon(const Translation3D& point) const;
	bool insidePolygonPlaneFrame(const Translation2D& projected_point) const;

	// polygon must be convex. contacts are rectangles.
	// TODO: split this fn up instead of switching on type
	bool contactInsidePolygon(const TransformationMatrix& transform, const std::string& contact_type) const;

	// roll is the rotation of the contact about ray
	TransformationMatrix projection(const Translation3D& origin, const Translation3D& ray, float roll,
								    const std::string& end_effector_type, bool valid_contact) const;

	// extract binary checking into another fn
	float distToBoundary(Translation3D point, float search_radius = 999/*, bool binary_checking = false*/) const;

	// inline std::shared_ptr<SurfaceContactPointGrid> get_contact_point_grid(){return contact_point_grid_;}
	std::shared_ptr<SurfaceContactPointGrid> contact_point_grid_;
	std::vector<Boundary> proj_boundaries_;
	std::vector<ContactRegion> contact_regions_; // constains a vector of contact regions belong to this surface

	void updateProjBoundaries(std::vector< std::shared_ptr<TrimeshSurface> > structures);

private:
	// boundaries historically called "approx_boundary"
	std::map<float, std::vector<float> > boundaries_; // discretized mapping from x coord to y bounds
	TransformationMatrix transformation_matrix_;
	TransformationMatrix inverse_transformation_matrix_;
	std::vector< std::pair<int, int> > edges_; // contains indices into vertices vector
	std::vector<Translation3D> vertices_; // historically called "boundaries"
	std::vector<Translation2D> proj_vertices_; // last vertex is same as first vertex, i.e. "closed loop"
	

	float min_proj_x_;
	float max_proj_x_;
	float min_proj_y_;
	float max_proj_y_;

	// nx * xo + ny * yo + nz * zo + c = 0
	float nx_;
	float ny_;
	float nz_;
	float c_;

	// center coordinates
	float xo_;
	float yo_;
	float zo_;

	float circum_radius_;

	TrimeshType type_;

	void updateCenter();
	void updateProjVertices();
	void updateApproxBoundary();

};

#endif
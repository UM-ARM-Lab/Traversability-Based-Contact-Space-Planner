// #include "TrimeshSurface.hpp"
#include "Utilities.hpp"

const float ERROR_TOLERANCE = .005;
const float SURFACE_SLICE_RESOLUTION = .005;


void TrimeshSurface::updateCenter()
{
	circum_radius_ = 0;
	float dist = 0;
	for(int i = 0; i < vertices_.size() - 1; ++i)
	{
		for(int j = i + 1; j < vertices_.size(); ++j)
		{
			dist = euclideanDistance3D(vertices_[i], vertices_[j]);
			if(dist/2 > circum_radius_)
			{
				circum_radius_ = dist / 2;
				xo_ = (vertices_[i][0] + vertices_[j][0]) / 2;
				yo_ = (vertices_[i][1] + vertices_[j][1]) / 2;
				zo_ = (vertices_[i][2] + vertices_[j][2]) / 2;
			}
		}
	}

	// std::cout << "Radius: " << circum_radius_ << std::endl;
}

void TrimeshSurface::updateProjVertices()
{
	min_proj_x_ = std::numeric_limits<float>::max();
	max_proj_x_ = std::numeric_limits<float>::min();

	min_proj_y_ = std::numeric_limits<float>::max();
	max_proj_y_ = std::numeric_limits<float>::min();

	proj_vertices_.clear();

	for(Translation3D const & vertex : vertices_)
	{
		Translation2D proj_vertex = projectionPlaneFrame(vertex);
		
		proj_vertices_.push_back(proj_vertex);
		
		min_proj_x_ = std::min(min_proj_x_, proj_vertex[0]);
		max_proj_x_ = std::max(max_proj_x_, proj_vertex[0]);

		min_proj_y_ = std::min(min_proj_y_, proj_vertex[1]);
		max_proj_y_ = std::max(max_proj_y_, proj_vertex[1]);

		// std::cout << "(" << vertex[0] << "," << vertex[1] << "," << vertex[2] << ") --> ";
		// std::cout << "(" << proj_vertex[0] << "," << proj_vertex[1] << ")" << std::endl;
	}

	// if(proj_vertices_.size())
	// { 
	// 	proj_vertices_.push_back(proj_vertices_.front()); // "close the loop"
	// }
}

void TrimeshSurface::updateApproxBoundary()
{
	for(std::pair<int, int> edge : edges_)
	{
		float start_x = proj_vertices_[edge.first][0];
		float start_y = proj_vertices_[edge.first][1];
		float end_x = proj_vertices_[edge.second][0];
		float end_y = proj_vertices_[edge.second][1];

		if(start_x == end_x)
		{
			continue;
		}

		if(start_x > end_x)
		{
			std::swap(start_x, end_x);
			std::swap(start_y, end_y);
		}

		int start_x_key = ceil((start_x - min_proj_x_) / SURFACE_SLICE_RESOLUTION);
		int end_x_key = (end_x - min_proj_x_) / SURFACE_SLICE_RESOLUTION;

		for(int i = start_x_key; i <= end_x_key; ++i)
		{
			float x_coord = i * SURFACE_SLICE_RESOLUTION + min_proj_x_;
			float y_coord = start_y + (x_coord - start_x) / (end_x - start_x) * (end_y - start_y);
			if(boundaries_.find(i) != boundaries_.end())
			{
				boundaries_[i].push_back(y_coord);
			}
			else
			{
				boundaries_[i] = {y_coord};
			}
		}

		// sort all y boundary points
		for(auto & boundary_pair : boundaries_)
		{
			sort(boundary_pair.second.begin(), boundary_pair.second.end());
		}
	}

	// std::cout << getId() << std::endl;
	// std::cout << "Edges: ";
	// for(std::pair<int, int> edge : edges_)
	// {
	// 	std::cout << "(" << edge.first << "," << edge.second << ")";
	// }
	// std::cout << std::endl;

	// std::cout << "Projected Vertices: " << std::endl;
	// for(auto & vertex: proj_vertices_)
	// {
	// 	std::cout << "(" << vertex[0] << "," << vertex[1] << ")" << std::endl;
	// }
	// std::cout << std::endl;

	// std::cout << "===============================================" << std::endl;


	// for(auto & boundary_pair : boundaries_)
	// {
	// 	sort(boundary_pair.second.begin(), boundary_pair.second.end());
	// 	std::cout << boundary_pair.first << ": ";
		
	// 	for(auto & boundary : boundary_pair.second)
	// 	{
	// 		std::cout<<boundary<<" ";
	// 	}
	// 	std::cout << std::endl;
	// }

	// int a;
	// std::cin >> a;

}

void TrimeshSurface::updateProjBoundaries(std::vector< std::shared_ptr<TrimeshSurface> > structures)
{
	// RAVELOG_INFO("A");
	
	proj_boundaries_.clear();

	float project_dist = 0.5;
	Translation2D origin(0,0);
	float min_z, max_z;
	int index1, index2;
	Translation3D vertex1, vertex2;
	Translation3D transformed_vertex;
	Translation3D transformed_vertex1, transformed_vertex2;
	Translation2D proj_vertex1, proj_vertex2;
	Translation3D intersection_point_3d;
	Translation2D intersection_point_2d;

	// RAVELOG_INFO("B");

    // first add its own boundary
	for(std::vector< std::pair<int, int> >::iterator edge_it = edges_.begin(); edge_it != edges_.end(); edge_it++)
	{
		index1 = edge_it->first;
		index2 = edge_it->second;
		Boundary new_boundary(proj_vertices_[index1], proj_vertices_[index2]);
		proj_boundaries_.push_back(new_boundary);
	}

	// RAVELOG_INFO("C");

	// collect all the surface boundaries and do the projection
	for(int st_index = 0; st_index < structures.size(); st_index++)
    {
		// RAVELOG_INFO("C-1");

        std::vector< std::shared_ptr<TrimeshSurface> >::iterator st_it = structures.begin() + st_index;

		if(getId() == (*st_it)->getId())
		{
			continue;
		}

		if(getType() != (*st_it)->getType())
		{
			continue;
		}

		// RAVELOG_INFO("C-2");

		// find the boundaries caused by edges
		std::vector<Translation2D> proj_vertices_from_other_surface;
		std::vector<Translation3D> transformed_vertices_from_other_surface;

		for(std::vector<Translation3D>::iterator vertex_it = (*st_it)->vertices_.begin(); vertex_it != (*st_it)->vertices_.end(); vertex_it++)
		{
			transformed_vertex = (inverse_transformation_matrix_ * vertex_it->homogeneous()).block(0,0,3,1);
			transformed_vertices_from_other_surface.push_back(transformed_vertex);

			proj_vertices_from_other_surface.push_back(projectionPlaneFrame(*vertex_it));
		}

		// RAVELOG_INFO("C-3");
		
        for(std::vector< std::pair<int, int> >::iterator edge_it = (*st_it)->edges_.begin(); edge_it != (*st_it)->edges_.end(); edge_it++)
		{
			// RAVELOG_INFO("C-3-1");

			index1 = edge_it->first;
			index2 = edge_it->second;

			transformed_vertex1 = transformed_vertices_from_other_surface[index1];
			transformed_vertex2 = transformed_vertices_from_other_surface[index2];

			max_z = std::max(transformed_vertex1(2), transformed_vertex2(2));
			min_z = std::min(transformed_vertex1(2), transformed_vertex2(2));

			// RAVELOG_INFO("C-3-2");

			// make sure the boundary is close enough in z
			if(max_z <= project_dist && max_z >= 0)
			{
				proj_vertex1 = proj_vertices_from_other_surface[index1];
				proj_vertex2 = proj_vertices_from_other_surface[index2];
				Boundary new_boundary(proj_vertex1, proj_vertex2);

				// RAVELOG_INFO("C-3-3");

				// make sure the boundary is close enough in xy
				if(new_boundary.getDistance(origin) <= circum_radius_)
				{
					// update one of the vertex to be the intersection point
					if(min_z < 0)
					{
						Translation2D intersection_proj_vertex;
						if(transformed_vertex1(2) > transformed_vertex2(2))
						{
							intersection_proj_vertex = ((-min_z)*proj_vertex1 + max_z*proj_vertex2)/(max_z-min_z);
							new_boundary = Boundary(proj_vertex1, intersection_proj_vertex);
						}
						else
						{
							intersection_proj_vertex = ((-min_z)*proj_vertex2 + max_z*proj_vertex1)/(max_z-min_z);
							new_boundary = Boundary(intersection_proj_vertex, proj_vertex2);
						}
					}
					
					proj_boundaries_.push_back(new_boundary);
				}

				// RAVELOG_INFO("C-3-4");
			}

			// RAVELOG_INFO("C-3-5");
		}

		// RAVELOG_INFO("C-4");

		// find the boundaries caused by the planar intersections
		// first check if they could possibly touch each other
		if(euclideanDistance3D(getCenter(), (*st_it)->getCenter()) < circum_radius_ + (*st_it)->circum_radius_)
		{
			// find planar intersection line
			Translation3D normal1 = getNormal();
			Translation3D normal2 = (*st_it)->getNormal();

			// if they are parallel, skip it.
			if(euclideanDistance3D(normal1, normal2) < 0.001)
			{
				continue;
			}

			std::vector<Translation2D> intersection_points;

			// project the other surface edges to the subject surface
			for(std::vector< std::pair<int, int> >::iterator edge_it = (*st_it)->edges_.begin(); edge_it != (*st_it)->edges_.end(); edge_it++)
			{
				index1 = edge_it->first;
			    index2 = edge_it->second;

				vertex1 = (*st_it)->vertices_[index1];
				vertex2 = (*st_it)->vertices_[index2];

				transformed_vertex1 = transformed_vertices_from_other_surface[index1];
				transformed_vertex2 = transformed_vertices_from_other_surface[index2];
				
				// check if the two vertices are in different side of the plane
				if(transformed_vertex1(2)*transformed_vertex2(2) < 0)
				{
					intersection_point_2d = projectionPlaneFrame(vertex1, (vertex2-vertex1).normalized());
					
					if(insidePolygonPlaneFrame(intersection_point_2d))
					{
						intersection_points.push_back(intersection_point_2d);
					}
				}
			}

			TransformationMatrix other_surface_inverse_transform = (*st_it)->getInverseTransform();
			std::vector<Translation3D> transformed_subject_vertices_to_other_surfaces;
			for(std::vector<Translation3D>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); vertex_it++)
			{
				transformed_vertex = (other_surface_inverse_transform * vertex_it->homogeneous()).block(0,0,3,1);
				transformed_subject_vertices_to_other_surfaces.push_back(transformed_vertex);
			}

			// project the subject surface edges to the other surface
			for(std::vector< std::pair<int, int> >::iterator edge_it = edges_.begin(); edge_it != edges_.end(); edge_it++)
			{
				index1 = edge_it->first;
			    index2 = edge_it->second;

				vertex1 = vertices_[index1];
				vertex2 = vertices_[index2];

				transformed_vertex1 = transformed_subject_vertices_to_other_surfaces[index1];
				transformed_vertex2 = transformed_subject_vertices_to_other_surfaces[index2];
				
				// check if the two vertices are in different side of the plane
				if(transformed_vertex1(2)*transformed_vertex2(2) < 0)
				{
					intersection_point_3d = (*st_it)->projectionGlobalFrame(vertex1, (vertex2-vertex1).normalized());
					
					if((*st_it)->insidePolygon(intersection_point_3d))
					{
						intersection_point_2d = projectionPlaneFrame(intersection_point_3d);
						intersection_points.push_back(intersection_point_2d);
					}
				}
			}

			if(intersection_points.size() >= 2) // should be either 0 or 2
			{
				Boundary new_boundary(intersection_points[0], intersection_points[1]);
				proj_boundaries_.push_back(new_boundary);
			}
			
		}

		// RAVELOG_INFO("C-5");

    }
	
}

/*** PUBLIC MEM FNS ***/
TrimeshSurface::TrimeshSurface(OpenRAVE::KinBodyPtr _kinbody, Eigen::Vector4f _plane_parameters,
								std::vector< std::pair<int, int> > _edges, std::vector<Translation3D> _vertices, TrimeshType _type, int _id):
								Structure(_kinbody,_id), 
								edges_(_edges), 
								vertices_(_vertices),
								contact_point_grid_(std::make_shared<SurfaceContactPointGrid>(min_proj_x_, max_proj_x_, min_proj_y_, max_proj_y_, SURFACE_CONTACT_POINT_RESOLUTION)),
								type_(_type)								
{

	// _plane_parameters.normalize();
	nx_ = _plane_parameters[0];
	ny_ = _plane_parameters[1];
	nz_ = _plane_parameters[2];
	c_ = _plane_parameters[3];

	updateCenter();

	assert(vertices_.size() != 0);

	float max_edge_length = 0;
	Translation3D edge_vector(0,0,0);
	for(std::vector< std::pair<int, int> >::iterator e_it = edges_.begin(); e_it != edges_.end(); e_it++)
	{
		Translation3D vertex_a = vertices_[e_it->first];
		Translation3D vertex_b = vertices_[e_it->second];

		float tmp_edge_length = (vertex_a-vertex_b).norm();

		if(tmp_edge_length > max_edge_length)
		{
			max_edge_length = tmp_edge_length;
			edge_vector = vertex_b - vertex_a;
		}
	}

	// see: http://fastgraph.com/makegames/3drotation/
	Translation3D vx = edge_vector.normalized(); // line of sight is from a vertex to center
	Translation3D vz = getNormal();
	Translation3D vy = vz.cross(vx);
	Translation3D center = getCenter();
	
	transformation_matrix_ = constructTransformationMatrix(vx[0], vy[0], vz[0], center[0], 
		                                                   vx[1], vy[1], vz[1], center[1], 
		                                                   vx[2], vy[2], vz[2], center[2]);	
	
	inverse_transformation_matrix_ = inverseTransformationMatrix(transformation_matrix_);

	// std::cout << "Center: (" << center[0] << "," << center[1] << "," << center[2] << ")" << std::endl;
	// std::cout << "Normal: (" << vz[0] << "," << vz[1] << "," << vz[2] << ")" << std::endl;
	
	updateProjVertices();
	updateApproxBoundary();

	contact_point_grid_->initializeParameters(min_proj_x_, max_proj_x_, min_proj_y_, max_proj_y_, SURFACE_CONTACT_POINT_RESOLUTION);
}

TrimeshSurface::TrimeshSurface(OpenRAVE::EnvironmentBasePtr _env, std::string _kinbody_name, Eigen::Vector4f _plane_parameters,
								std::vector< std::pair<int, int> > _edges, std::vector<Translation3D> _vertices, TrimeshType _type, int _id):
								Structure(_env->GetKinBody(_kinbody_name),_id), 
								edges_(_edges), 
								vertices_(_vertices),
								contact_point_grid_(std::make_shared<SurfaceContactPointGrid>(min_proj_x_, max_proj_x_, min_proj_y_, max_proj_y_, SURFACE_CONTACT_POINT_RESOLUTION)),
								type_(_type)
{

	// _plane_parameters.normalize();
	nx_ = _plane_parameters[0];
	ny_ = _plane_parameters[1];
	nz_ = _plane_parameters[2];
	c_ = _plane_parameters[3];

	updateCenter();

	assert(vertices_.size() != 0);

	float max_edge_length = 0;
	Translation3D edge_vector(0,0,0);
	for(std::vector< std::pair<int, int> >::iterator e_it = edges_.begin(); e_it != edges_.end(); e_it++)
	{
		Translation3D vertex_a = vertices_[e_it->first];
		Translation3D vertex_b = vertices_[e_it->second];

		float tmp_edge_length = (vertex_a-vertex_b).norm();

		if(tmp_edge_length > max_edge_length)
		{
			max_edge_length = tmp_edge_length;
			edge_vector = vertex_b - vertex_a;
		}
	}

	// see: http://fastgraph.com/makegames/3drotation/
	Translation3D vx = edge_vector.normalized(); // line of sight is from a vertex to center
	Translation3D vz = getNormal();
	Translation3D vy = vz.cross(vx);
	Translation3D center = getCenter();

	transformation_matrix_ = constructTransformationMatrix(vx[0], vy[0], vz[0], center[0], 
														   vx[1], vy[1], vz[1], center[1], 
														   vx[2], vy[2], vz[2], center[2]);
		
	inverse_transformation_matrix_ = inverseTransformationMatrix(transformation_matrix_);

	// std::cout << "Center: (" << center[0] << "," << center[1] << "," << center[2] << ")" << std::endl;
	// std::cout << "Normal: (" << vz[0] << "," << vz[1] << "," << vz[2] << ")" << std::endl;

	updateProjVertices();
	updateApproxBoundary();

	contact_point_grid_->initializeParameters(min_proj_x_, max_proj_x_, min_proj_y_, max_proj_y_, SURFACE_CONTACT_POINT_RESOLUTION);
}

// OpenRAVE::TriMesh TrimeshSurface::get_openrave_trimesh() const
// {
// 	OpenRAVE::TriMesh ret_tm;
// 	ret_tm.vertices = vertices_;
// 	ret_tm.indices = {0, 1, 2, 2, 3, 0}; // generalize this to non-rectangular tri meshes
// 	return ret_tm;
// }

void TrimeshSurface::transform_data(TransformationMatrix transform)
{
	transformation_matrix_ = transformation_matrix_ * transform;
	inverse_transformation_matrix_ = inverse_transformation_matrix_ * transform.inverse();

	Translation3D transformed_normal = (transform * getNormal().homogeneous()).block(0,0,3,1);
	nx_ = transformed_normal[0];
	ny_ = transformed_normal[1];
	nz_ = transformed_normal[2];

	Translation3D transformed_center = (transform * getCenter().homogeneous()).block(0,0,3,1);
	xo_ = transformed_center[0];
	yo_ = transformed_center[1];
	zo_ = transformed_center[2];

	c_ = -(nx_ * vertices_[0][0] + ny_ * vertices_[0][1] + nz_ * vertices_[0][2]);

	for(int i = 0; i < vertices_.size(); ++i)
	{
		vertices_[i] = (transform * vertices_[i].homogeneous()).block(0,0,3,1);
	}

	updateProjVertices();
}

// returns 2D point projected in plane frame. This assumes the "ray" is the surface normal.
Translation2D TrimeshSurface::projectionPlaneFrame(const Translation3D& start_point) const
{
	Translation3D proj_point = (inverse_transformation_matrix_ * start_point.homogeneous()).block(0,0,3,1);

	return Translation2D(proj_point[0],proj_point[1]);
}

Translation2D TrimeshSurface::projectionPlaneFrame(const Translation3D& start_point, const Translation3D& ray) const
{
	Translation3D proj_point_global_frame = projectionGlobalFrame(start_point, ray);

	if(isValidPosition(proj_point_global_frame))
	{
		Translation3D proj_point_surface_frame = (inverse_transformation_matrix_ * proj_point_global_frame.homogeneous()).block(0,0,3,1);

		return Translation2D(proj_point_surface_frame[0], proj_point_surface_frame[1]);
	}
	else
	{
		return Translation2D(-99.0,-99.0);
	}
	
}

Translation3D TrimeshSurface::getGlobalPosition(const Translation2D& point) const
{
	Translation3D point3D(point[0],point[1],0);
	return (getTransform() * point3D.homogeneous()).block(0,0,3,1);
}

Translation3D TrimeshSurface::projectionGlobalFrame(const Translation3D& start_point, const Translation3D& ray) const
{
	float cosine = getNormal().dot(ray);

	if(fabs(cosine) < 0.001)
	{
		return Translation3D(-99.0,-99.0,-99.0);
	}

	float t = (-c_ - getNormal().dot(start_point)) / cosine; // double check asserts

	if(t >= -0.001)
	{
		Translation3D p = start_point + t * ray;
		return p;
	}
	else
	{
		return Translation3D(-99.0,-99.0,-99.0);
	}
	
}

bool TrimeshSurface::insidePolygon(const Translation3D& point) const
{
	float x = point[0]; float y = point[1]; float z = point[2];

	if(abs(nx_ * x + ny_ * y + nz_ * z + c_) > ERROR_TOLERANCE)
	{
		return false;
	}

	if (euclideanDistance3D(point, getCenter()) >= circum_radius_)
	{
		return false;
	}

	Translation2D projected_point = projectionPlaneFrame(point);
	return insidePolygonPlaneFrame(projected_point);
}

bool TrimeshSurface::insidePolygonPlaneFrame(const Translation2D& projected_point) const
{
	if(projected_point[0] > max_proj_x_ || projected_point[0] < min_proj_x_ || 
	   projected_point[1] > max_proj_y_ || projected_point[1] < min_proj_y_)
	{
		return false;
	}

	int query_x = (projected_point[0] - min_proj_x_) / SURFACE_SLICE_RESOLUTION;
	auto y_bounds_it = boundaries_.find(query_x);

	if(y_bounds_it == boundaries_.end())
	{
		return false;
	}

	int pass_boundary_count = 0;

	// considers points "on border" to be within the frame
	auto bound_it = lower_bound(y_bounds_it->second.begin(), y_bounds_it->second.end(), projected_point[1]);

	return distance(y_bounds_it->second.begin(), bound_it) % 2 == 1;
}

// polygon must be convex. contacts are rectangles.
// TODO: split this fn up instead of switching on type
bool TrimeshSurface::contactInsidePolygon(const TransformationMatrix& transform, const std::string& contact_type) const
{
	float h;
	float w;
	std::vector<Translation3D> contact_vertices(4);

	if(contact_type == "foot")
	{
		h = FOOT_HEIGHT / 2;
		w = FOOT_WIDTH / 2;
		contact_vertices[0] = Translation3D(h, w, 0);
		contact_vertices[1] = Translation3D(h, -w, 0);
		contact_vertices[2] = Translation3D(-h, w, 0);
		contact_vertices[3] = Translation3D(-h, -w, 0);
	}
	else if(contact_type == "hand")
	{
		h = HAND_HEIGHT / 2;
		w = HAND_WIDTH / 2;
		contact_vertices[0] = Translation3D(0, h, w);
		contact_vertices[1] = Translation3D(0, h, -w);
		contact_vertices[2] = Translation3D(0, -h, w);
		contact_vertices[3] = Translation3D(0, -h, -w);
	}
	else
	{
		// unexpected contact.
		// add exceptions to code? -> slight performance decrease.
		return false;
	}

	// check if transformed contact vertex is inside polygon
	for(Translation3D & vertex : contact_vertices)
	{
		if(!insidePolygon((transform * vertex.homogeneous()).block(0,0,3,1)))
		{
			return false;
		}
	}

	return true;
}

// roll is the rotation of the contact about ray
TransformationMatrix TrimeshSurface::projection(const Translation3D& origin, const Translation3D& ray, float roll, const std::string & end_effector_type, bool valid_contact) const
{
	Translation3D translation = projectionGlobalFrame(origin, ray);

	Translation3D cx, cy, cz;

	if(end_effector_type == "foot")
	{
		cz = getNormal();
		cx = Translation3D(cos(roll * M_PI / 180), sin(roll * M_PI / 180), 0);
		cy = cz.cross(cx).normalized();
		cx = cy.cross(cy);
	}

	TransformationMatrix ret_transform;
	
	ret_transform = constructTransformationMatrix(cx[0],cy[0],cz[0],translation[0],
	                                                cx[1],cy[1],cz[1],translation[1],
													cx[2],cy[2],cz[2],translation[2]);
	
	return ret_transform;
}

// extract binary checking into another function
float TrimeshSurface::distToBoundary(Translation3D point, float search_radius/*, bool binary_checking = false*/) const
{
	Translation2D projected_point = projectionPlaneFrame(point);
	float dist = search_radius; // assume point is inside boundaries_

	int upper_x = std::min(ceil((projected_point[0] + search_radius - min_proj_x_) / SURFACE_SLICE_RESOLUTION), (double)boundaries_.size());
	int middle_x = (projected_point[0] - min_proj_x_) / SURFACE_SLICE_RESOLUTION;
	int lower_x = std::max((projected_point[0] - search_radius - min_proj_x_) / SURFACE_SLICE_RESOLUTION, (float)0.0);

	int half_interval_x = std::min(upper_x - middle_x, middle_x - lower_x);

	if((boundaries_.size() - 1 - middle_x) * SURFACE_SLICE_RESOLUTION < dist)
	{
		dist = (boundaries_.size() - 1 - middle_x) * SURFACE_SLICE_RESOLUTION;
		// if(binary_checking) return false;
	}

	if(middle_x * SURFACE_SLICE_RESOLUTION < dist)
	{
		dist = middle_x * SURFACE_SLICE_RESOLUTION;
		// if(binary_checking) return false;
	}

	std::vector<float> x_slices(2 * half_interval_x);
	std::iota(x_slices.begin(), x_slices.end(), middle_x - half_interval_x);


	sort(x_slices.begin(), x_slices.end(), [&projected_point, this](int a, int b) {
		return fabs(SURFACE_SLICE_RESOLUTION * a + min_proj_x_ - projected_point[0]) <
				fabs(SURFACE_SLICE_RESOLUTION * b + min_proj_x_ - projected_point[0]);
	});

	for(float ix : x_slices)
	{
		auto y_bounds = boundaries_.at(ix);
		float x_coord = SURFACE_SLICE_RESOLUTION * ix + min_proj_x_;

		if(abs(x_coord - projected_point[0]) > dist)
		{
			continue;
		}

		for(int i = 0; i < y_bounds.size() - 1; ++i)
		{
			float y_bound = y_bounds[i];
			float next_y_bound = y_bounds[i + 1];
			float dist_to_slice_bound = std::min(euclideanDistance2D(Translation2D(x_coord, y_bound), projected_point), euclideanDistance2D(Translation2D(x_coord, next_y_bound), projected_point));

			if(dist_to_slice_bound < dist)
			{
				dist = dist_to_slice_bound;
				// if(binary_checking) return false;
			}
		}
	}

	// if(binary_checking) return true;
	return dist;
}
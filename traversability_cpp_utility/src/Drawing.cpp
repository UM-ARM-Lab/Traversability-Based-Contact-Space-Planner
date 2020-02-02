#include "Utilities.hpp"

// using namespace OpenRAVE;

DrawingHandler::DrawingHandler(OpenRAVE::EnvironmentBasePtr _penv):penv(_penv)
{
	foot_corners.resize(4);
	foot_corners[0] = OpenRAVE::RaveVector<OpenRAVE::dReal>(FOOT_HEIGHT/2,FOOT_WIDTH/2,0.01);
	foot_corners[1] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-FOOT_HEIGHT/2,FOOT_WIDTH/2,0.01);
	foot_corners[2] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-FOOT_HEIGHT/2,-FOOT_WIDTH/2,0.01);
	foot_corners[3] = OpenRAVE::RaveVector<OpenRAVE::dReal>(FOOT_HEIGHT/2,-FOOT_WIDTH/2,0.01);

	hand_corners.resize(4);
	hand_corners[0] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-0.01,HAND_HEIGHT/2,HAND_WIDTH/2);
	hand_corners[1] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-0.01,-HAND_HEIGHT/2,HAND_WIDTH/2);
	hand_corners[2] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-0.01,-HAND_HEIGHT/2,-HAND_WIDTH/2);
    hand_corners[3] = OpenRAVE::RaveVector<OpenRAVE::dReal>(-0.01,HAND_HEIGHT/2,-HAND_WIDTH/2);
}

void DrawingHandler::ClearHandler()
{
	graphptrs.clear();
}

// void DrawingHandler::DrawBodyPath(Node* current) // Draw the upperbody path in thr door planning, postpone this implementation.(DrawPaths)
// {

// }

void DrawingHandler::DrawGridPath() // Draw the Dijkstra grid path, postpone implementation.
{

}

// void DrawingHandler::DrawContactPath(Node* current) // Draw the contact path given the final state(DrawStances)
// {
//     Node* c = current;
//     while(c != NULL)
//     {
//     	DrawContacts(c);    	
//         c = c->get_parent();
//     }
// }
// void DrawingHandler::DrawContacts(Node* node) // Draw the contacts of one node(DrawStance)
// {
// 	// draw left foot pose
//     OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> left_foot_transform = get_SO3(node->get_left_foot());
//     std::vector< OpenRAVE::RaveVector<OpenRAVE::dReal> > transformed_left_foot_corners(4);

//     for(unsigned int i = 0; i < transformed_left_foot_corners.size(); i++)
//     {
//     	transformed_left_foot_corners[i] = left_foot_transform*foot_corners[i];
//     }

//     float left_foot_corners0_x_float = (float)transformed_left_foot_corners[0].x;
//     graphptrs.push_back(penv->drawlinestrip(&(left_foot_corners0_x_float ),transformed_left_foot_corners.size(),sizeof(transformed_left_foot_corners[0]),5,OpenRAVE::RaveVector<float>(1,0,0,0)));

//     // draw right foot pose
//     OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> right_foot_transform = get_SO3(node->get_right_foot());
//     std::vector< OpenRAVE::RaveVector<OpenRAVE::dReal> > transformed_right_foot_corners(4);

//     for(unsigned int i = 0; i < transformed_right_foot_corners.size(); i++)
//     {
//     	transformed_right_foot_corners[i] = right_foot_transform*foot_corners[i];
//     }

//     float right_foot_corners0_x_float = (float)transformed_right_foot_corners[0].x;
//     graphptrs.push_back(penv->drawlinestrip(&(right_foot_corners0_x_float),transformed_right_foot_corners.size(),sizeof(transformed_right_foot_corners[0]),5,OpenRAVE::RaveVector<float>(0,1,0,0)));

//     // draw left hand pose
//     if(node->get_left_hand().x != -99.0)
//     {
//         OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> left_hand_transform = get_SO3(node->get_left_hand());
//         std::vector< OpenRAVE::RaveVector<OpenRAVE::dReal> > transformed_left_hand_corners(4);

//         for(unsigned int i = 0; i < transformed_left_hand_corners.size(); i++)
//         {
//         	transformed_left_hand_corners[i] = left_hand_transform*hand_corners[i];
//         }

//         float left_hand_corners0_x_float = (float)transformed_left_hand_corners[0].x;
//         graphptrs.push_back(penv->drawlinestrip(&(left_hand_corners0_x_float),transformed_left_hand_corners.size(),sizeof(transformed_left_hand_corners[0]),5,OpenRAVE::RaveVector<float>(0,0,1,0)));
//     }

//     // draw right hand pose
//     if(node->get_right_hand().x != -99.0)
//     {
//         OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> right_hand_transform = get_SO3(node->get_right_hand());
//         std::vector< OpenRAVE::RaveVector<OpenRAVE::dReal> > transformed_right_hand_corners(4);

//         for(unsigned int i = 0; i < transformed_right_hand_corners.size(); i++)
//         {
//         	transformed_right_hand_corners[i] = right_hand_transform*hand_corners[i];
//         }

//         float right_hand_corners0_x_float = (float)transformed_right_hand_corners[0].x;
//         graphptrs.push_back(penv->drawlinestrip(&(right_hand_corners0_x_float),transformed_right_hand_corners.size(),sizeof(transformed_right_hand_corners[0]),5,OpenRAVE::RaveVector<float>(1,1,0,0)));
//     }
// }
// void DrawingHandler::DrawContact(enum contact_type,contact_transform); // Draw one contact.(DrawContact)

void DrawingHandler::DrawLocation(OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> transform, OpenRAVE::RaveVector<float> color) // Draw a point at the location(DrawLocation)
{
	float trans_x_float = (float)transform.trans[0];
	graphptrs.push_back(penv->plot3(&(trans_x_float), 1, 0, 15, color));
}

void DrawingHandler::DrawLocation(OpenRAVE::RaveVector<OpenRAVE::dReal> location, OpenRAVE::RaveVector<float> color) // Draw a point at the location(DrawLocation)
{
	float trans_x_float = (float)location[0];
	graphptrs.push_back(penv->plot3(&(trans_x_float), 1, 0, 15, color));
}

void DrawingHandler::DrawTransform(OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> transform) // Draw the transform in 3 axes(DrawOrientation)
{
	OpenRAVE::RaveVector<OpenRAVE::dReal> from_vec = transform.trans;
	OpenRAVE::RaveVector<OpenRAVE::dReal> to_vec_x = from_vec + 0.2 * OpenRAVE::RaveVector<OpenRAVE::dReal>(transform.m[0],transform.m[4],transform.m[8]);
	OpenRAVE::RaveVector<OpenRAVE::dReal> to_vec_y = from_vec + 0.2 * OpenRAVE::RaveVector<OpenRAVE::dReal>(transform.m[1],transform.m[5],transform.m[9]);
	OpenRAVE::RaveVector<OpenRAVE::dReal> to_vec_z = from_vec + 0.2 * OpenRAVE::RaveVector<OpenRAVE::dReal>(transform.m[2],transform.m[6],transform.m[10]);

	graphptrs.push_back(penv->drawarrow(from_vec, to_vec_x, 0.005, OpenRAVE::RaveVector<float>(1, 0, 0)));
    graphptrs.push_back(penv->drawarrow(from_vec, to_vec_y, 0.005, OpenRAVE::RaveVector<float>(0, 1, 0)));
    graphptrs.push_back(penv->drawarrow(from_vec, to_vec_z, 0.005, OpenRAVE::RaveVector<float>(0, 0, 1)));
}

void DrawingHandler::DrawManipulatorPoses(OpenRAVE::RobotBasePtr robot) // Draw the manipulator poses given robot object(DrawManipulatorPoses)
{
	std::vector< boost::shared_ptr<OpenRAVE::RobotBase::Manipulator> > manipulators = robot->GetManipulators();

	for(unsigned int i = 0; i < manipulators.size(); i++)
	{
		boost::shared_ptr<OpenRAVE::RobotBase::Manipulator> manipulator = manipulators[i];
		DrawTransform(OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal>(manipulator->GetTransform()));
	}
}

void DrawingHandler::DrawGoalRegion(OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> transform, double radius) // Draw the region with given transform and radius.(DrawRegion)
{
	OpenRAVE::RaveVector<OpenRAVE::dReal> center = transform.trans;
	OpenRAVE::RaveVector<OpenRAVE::dReal> x_vector = OpenRAVE::RaveVector<OpenRAVE::dReal>(transform.m[0],transform.m[4],transform.m[8]);
	OpenRAVE::RaveVector<OpenRAVE::dReal> y_vector = OpenRAVE::RaveVector<OpenRAVE::dReal>(transform.m[1],transform.m[5],transform.m[9]);

	DrawRegion(center, OpenRAVE::RaveVector<OpenRAVE::dReal>(0,0,1), radius, 5.0); // Draw the region with given center, normal and radius.(DrawContactRegion)

	std::vector< OpenRAVE::RaveVector<OpenRAVE::dReal> > arrow_points(5);

	arrow_points[0] = center - radius*2.0/3.0 * x_vector;
	arrow_points[1] = center + radius*2.0/3.0 * x_vector;
	arrow_points[2] = center + radius/2.0 * y_vector;
	arrow_points[3] = center + radius*2.0/3.0 * x_vector;
	arrow_points[4] = center - radius/2.0 * y_vector;

	float arrow_point_x_float = (float)arrow_points[0].x;

	graphptrs.push_back(penv->drawlinestrip(&(arrow_point_x_float),arrow_points.size(),sizeof(arrow_points[0]),5.0,OpenRAVE::RaveVector<float>(0,0,0,0)));
}

void DrawingHandler::DrawRegion(OpenRAVE::RaveVector<OpenRAVE::dReal> center, OpenRAVE::RaveVector<OpenRAVE::dReal> normal, double radius, float line_width) // Draw the region with given center, normal and radius.(DrawContactRegion)
{
	OpenRAVE::RaveVector<OpenRAVE::dReal> x_vector;
	if(normal.x == 0 && normal.y == 0)
	{
        x_vector = OpenRAVE::RaveVector<OpenRAVE::dReal>(1,0,0);
	}
    else
    {
    	x_vector = OpenRAVE::RaveVector<OpenRAVE::dReal>(normal.y,-normal.x,0);
        x_vector = x_vector.normalize3();
    }

    OpenRAVE::RaveVector<OpenRAVE::dReal> y_vector = normal.cross(x_vector);

    std::vector<OpenRAVE::RaveVector<float> >* region_boundary_points_float = new std::vector<OpenRAVE::RaveVector<float > >;
    // std::vector< OpenRAVE::RaveVector<float> > region_boundary_points_float;
    region_boundary_points_float->resize(37);
    OpenRAVE::RaveVector<OpenRAVE::dReal> region_boundary_point;

	for(unsigned int i = 0; i < 37; i++)
	{
		region_boundary_point = center + std::cos(i*10*(M_PI / 180))*radius*x_vector + std::sin(i*10*(M_PI / 180))*radius*y_vector;
		(*region_boundary_points_float)[i] = {region_boundary_point.x, region_boundary_point.y,
									          region_boundary_point.z, region_boundary_point.w}; // truncate OpenRAVE::dReals to floats
	}

	region_boundary_pointers.push_back(region_boundary_points_float);

	graphptrs.push_back(penv->drawlinestrip((float *)region_boundary_points_float->data(),region_boundary_points_float->size(),sizeof((*region_boundary_points_float)[0]),line_width,OpenRAVE::RaveVector<float>(0,0,0,1)));
	graphptrs.push_back(penv->drawarrow(center, center + 0.1 * normal, 0.005, OpenRAVE::RaveVector<float>(1,0,0, 1)));
}

void DrawingHandler::DrawLineSegment(Translation3D from_vec, Translation3D to_vec, std::array<float,4> color)
{
    OpenRAVE::RaveVector<OpenRAVE::dReal> from_vec_ravevector = OpenRAVE::RaveVector<OpenRAVE::dReal>(from_vec[0],from_vec[1],from_vec[2]);
    OpenRAVE::RaveVector<OpenRAVE::dReal> to_vec_ravevector = OpenRAVE::RaveVector<OpenRAVE::dReal>(to_vec[0],to_vec[1],to_vec[2]);
    OpenRAVE::RaveVector<float> color_ravevector = OpenRAVE::RaveVector<float>(color[0],color[1],color[2],color[3]);

    DrawLineSegment(from_vec_ravevector, to_vec_ravevector, color_ravevector);
}

void DrawingHandler::DrawLineSegment(OpenRAVE::RaveVector<OpenRAVE::dReal> from_vec, OpenRAVE::RaveVector<OpenRAVE::dReal> to_vec, OpenRAVE::RaveVector<float> color) // Draw a line segment given two ends(DrawLineStrips)
{
    std::vector<float> line_endpoints = {(float)from_vec[0],(float)from_vec[1],(float)from_vec[2],(float)to_vec[0],(float)to_vec[1],(float)to_vec[2]};
	graphptrs.push_back(penv->drawlinestrip(&(line_endpoints[0]),line_endpoints.size()/3,sizeof(line_endpoints[0])*3,3.0,color));
}

// void DrawingHandler::DrawSurface(TrimeshSurface trimesh) // Draw the trimesh surface.(DrawSurface)
// {
// 	float r = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);
// 	float g = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);
// 	float b = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);

// 	float total_rgb = std::sqrt(r*r+g*g+b*b);
// 	r = r/total_rgb;
// 	g = g/total_rgb;
// 	b = b/total_rgb;

// 	// DrawOrientation(trimesh.transform_matrix);

// 	// for boundary in struct.boundaries:

// 	// 	boundaries_point = np.zeros((2,3),dtype=float)
// 	// 	boundaries_point[0:1,:] = np.atleast_2d(np.array(struct.vertices[boundary[0]]))
// 	// 	boundaries_point[1:2,:] = np.atleast_2d(np.array(struct.vertices[boundary[1]]))

// 	// 	draw_handles.append(env.drawlinestrip(points = boundaries_point,linewidth = 5.0,colors = np.array((r,g,b))))

// 	// graphptrs.push_back(penv->drawtrimesh())
// 	// _penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,OpenRAVE::RaveVector<float>(1,0.5,0.5,1))
// 	// draw_handles.append(env.drawtrimesh(trimesh.kinbody->GetLinks()[0].GetCollisionData().vertices,struct.kinbody.GetLinks()[0].GetCollisionData().indices,OpenRAVE::RaveVector<float>(r,g,b,1.0)))

// }

// void DrawingHandler::DrawObjectPath(Node* current) // Draw the manipulated object path, postpone implementation.(DrawObjectPath)
// {

// }

DrawingHandler::~DrawingHandler()
{
    for(int i = 0; i < region_boundary_pointers.size(); ++i)
    {
		delete region_boundary_pointers[i];
	}
}
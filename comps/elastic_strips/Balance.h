#ifndef  BALANCE_H
#define  BALANCE_H

#define GMPRATIONAL 1

#include <cdd/setoper.h>
#include <cdd/cdd.h>

enum BalanceMode {
    BALANCE_NONE,
    BALANCE_SUPPORT_POLYGON,
    BALANCE_GIWC
};

#define CONE_DISCRETIZATION_RESOLUTION 4
#define CONE_COMPUTATION_PRECISION 1e4


class Balance
{
	public:
		Balance(){};
		Balance(RobotBasePtr r, std::vector<string> s_links, Vector p_scale, Vector p_trans);
		Balance(RobotBasePtr r, Vector g, std::vector<string> s_manips, std::vector<dReal> s_mus);
		~Balance(){};

		class Point2D
		{
		    public:
		        double x;
		        double y;
		};

		void RefreshBalanceParameters(std::vector<dReal> q_new); // call it after the robot is in a new configuration, and before calling CheckSupport. Takes new config.
		void RefreshBalanceParameters(std::vector<dReal> q_new, vector<string> s_links_manips, std::map<string,NEWMAT::Matrix>& giwc_database);
		bool CheckSupport(Vector center);
    	
	private:
		BalanceMode _balance_mode;
		RobotBasePtr _Robot;

		//support polygon
		std::vector<string> _supportlinks;
		Vector _polyscale;
		Vector _polytrans;
		std::vector<dReal> _supportpolyx;
		std::vector<dReal> _supportpolyy;

		//GIWC
		Vector _gravity;
		std::vector<string> _support_manips;
		std::vector<dReal> _support_mus;
		NEWMAT::Matrix _giwc;

		//map index to support link / support manip
		// std::map<int,string> _manip_index_name_map;
		std::map<string,int> _manip_name_index_map;

		std::map<string,NEWMAT::Matrix> _computed_contact_surface_cones;

		void InitializeBalanceParameters();

		void GetSupportPolygon();
		int convexHull2D(coordT* pointsIn, int numPointsIn, coordT** pointsOut, int* numPointsOut);
		int convexHull6D(coordT* pointsIn, int numPointsIn, std::vector< std::vector<double> >& facet_coefficients);
		Balance::Point2D compute2DPolygonCentroid(const Balance::Point2D* vertices, int vertexCount);
		
		void GetGIWC();
		
		NEWMAT::ReturnMatrix GetGIWCSpanForm();	
		NEWMAT::ReturnMatrix GetSurfaceCone(string& manipname, dReal mu);
		void GetSupportPointsForLink(RobotBase::LinkPtr p_link, OpenRAVE::Vector tool_dir, Transform result_tf, std::vector<Vector>& contacts);
		std::vector<Vector> GetSupportPoints(RobotBase::ManipulatorPtr p_manip);
		void GetFrictionCone(OpenRAVE::Vector &center, OpenRAVE::Vector &direction, dReal mu, NEWMAT::Matrix *mat, int offset_r, int offset_c, Transform temp_tf);
		void GetASurf(RobotBase::ManipulatorPtr p_manip, Transform cone_tf, NEWMAT::Matrix *mat, int offset_r);
		void GetAStance(Transform cone_tf, NEWMAT::Matrix* mat, int offset_r);


};

#endif

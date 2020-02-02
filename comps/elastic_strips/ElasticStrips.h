#ifndef  ELASTICSTRIPS_H
#define  ELASTICSTRIPS_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>
// #include "ContactRegion.h"

class ElasticStrips : public ModuleBase
{
  public:
    ElasticStrips(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RunElasticStrips",boost::bind(&ElasticStrips::RunElasticStrips,this,_1,_2),
                        "Run the Elastic Strips Planner");

        _esEnv = penv;

    }
    virtual ~ElasticStrips() {}

    int RunElasticStrips(ostream& sout, istream& sinput);

  private:

    void SetActiveRobots();

    void InitPlan(boost::shared_ptr<ESParameters> params);
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj);
    void LoadContactRegions(); // new elastic strips
    void DecideContactConsistentTransform(TrajectoryBasePtr ptraj);
    int FindNearestContactRegion();
    void FindContactRegions();
    void FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj);
    Transform ForwardKinematics(std::vector<dReal> qs, RobotBasePtr robot, string manip_name);
    dReal TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ);

    void GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point, size_t w);
    void GetInternalVector(Vector& internal_vector, TrajectoryBasePtr ptraj, RobotBasePtr robot, string control_link, size_t w);

    void UpdateZRPYandXYJacobianandStep(Transform taskframe_in, size_t w);
    void UpdateCOGJacobianandStep(Transform taskframe_in, size_t w);
    void UpdateOAJacobianandStep(Transform taskframe_in, TrajectoryBasePtr ptraj, size_t w, bool& bInCollision);
    void UpdatePCJacobianandStep(Transform taskframe_in, size_t w);
    void UpdateINTJacobianandStep(Transform taskframe_in, TrajectoryBasePtr ptraj, size_t w);

    void QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi);
    int invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    void RemoveBadJointJacobianCols(NEWMAT::Matrix& J, size_t w);

    NEWMAT::ColumnVector CalculateStep();
    void PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement);

    int _numdofs;
    int _num_robots;

    EnvironmentBasePtr _esEnv;
    RobotBasePtr _esRobot;
    std::vector<RobotBasePtr> _esRobotVec; // the first robot in this vector is _esRobot
    std::vector<dReal> _lowerLimit, _upperLimit;

    std::stringstream _outputstream;

    boost::shared_ptr<ESParameters> _parameters;

    std::vector<ContactRegion> _contact_regions;

    std::map< int, ContactRegion > _nearest_contact_regions; // <manips group, contact region>
    std::map< int, ContactManipGroup > _contact_manips_group; // <group index, <manip name, contact-consistent transform, manip of (manip name) in the wp index belongs to this group> >
    std::map< size_t, std::map<string,int> > _waypoint_manip_group_map; // <waypoint index, <manip_name,contact consistent transform> >

    std::vector<int> _start_contact_group_index;
    std::vector<int> _goal_contact_group_index;

    string _strRobotName; // name of the active robot

    std::vector<Vector> _cogtargs;
    std::vector<Vector> _curcogs;
    std::vector<std::shared_ptr<Balance> > _balance_checkers;

    std::vector<bool> _stable_waypoint;

    std::vector<std::map<string,int> > _contact_consistent_region; // <waypoint index, <manip_name,cc manip position> >
    std::vector< std::pair<string,Vector> > _contact_consistent_manip_translation; // store the contact consistent point/translation in each iteration, index: contact region index, <manip name, position of the link>

    std::vector<NEWMAT::Matrix> _Jp;
    std::vector<NEWMAT::Matrix> _Jp0;

    std::vector<NEWMAT::Matrix> _Jr;
    std::vector<NEWMAT::Matrix> _Jr0;

    std::vector<NEWMAT::Matrix> _Jr_proper;

    std::vector<NEWMAT::Matrix> _Jr_quat;

    std::vector<NEWMAT::Matrix> _tooltm;
    std::vector<TransformMatrix> _TMtool;


    std::vector<NEWMAT::Matrix> _tasktm;
    std::vector<NEWMAT::Matrix> _E_rpy;
    std::vector<NEWMAT::Matrix> _E_rpy_inv;

    std::vector<TransformMatrix> _TMtask;

    std::vector<NEWMAT::Matrix> _JOA;
    std::vector<NEWMAT::Matrix> _JOAplus;
    std::vector<NEWMAT::ColumnVector> _doa;
    
    std::vector<NEWMAT::Matrix> _JZ;
    std::vector<NEWMAT::Matrix> _JRPY;

    std::vector<NEWMAT::Matrix> _JZRPY;
    std::vector<NEWMAT::Matrix> _JZRPYplus;
    std::vector<NEWMAT::ColumnVector> _dzrpy;

    std::vector<NEWMAT::Matrix> _JXY;
    std::vector<NEWMAT::Matrix> _JXYplus;
    std::vector<NEWMAT::ColumnVector> _dxy;

    std::vector<NEWMAT::Matrix> _JXYZRPY;
    std::vector<NEWMAT::Matrix> _JXYZRPYplus;
    std::vector<NEWMAT::ColumnVector> _dxyzrpy;
    
    std::vector<NEWMAT::Matrix> _JCOG;
    std::vector<NEWMAT::Matrix> _JCOGplus;
    std::vector<NEWMAT::ColumnVector> _dcog;

    std::vector<NEWMAT::Matrix> _JPC;
    std::vector<NEWMAT::Matrix> _JPCplus;
    std::vector<NEWMAT::ColumnVector> _dpc;

    std::vector<NEWMAT::Matrix> _JINT;
    std::vector<NEWMAT::Matrix> _JINTplus;
    std::vector<NEWMAT::ColumnVector> _dint;
    
    std::vector<NEWMAT::SymmetricMatrix> _Moa;
    std::vector<NEWMAT::SymmetricMatrix> _Moainv;

    std::vector<NEWMAT::SymmetricMatrix> _Mzrpy;
    std::vector<NEWMAT::SymmetricMatrix> _Mzrpyinv;

    std::vector<NEWMAT::SymmetricMatrix> _Mxy;
    std::vector<NEWMAT::SymmetricMatrix> _Mxyinv;

    std::vector<NEWMAT::SymmetricMatrix> _Mxyzrpy;
    std::vector<NEWMAT::SymmetricMatrix> _Mxyzrpyinv;

    std::vector<NEWMAT::SymmetricMatrix> _Mcog;
    std::vector<NEWMAT::SymmetricMatrix> _Mcoginv;

    std::vector<NEWMAT::SymmetricMatrix> _Mpc;
    std::vector<NEWMAT::SymmetricMatrix> _Mpcinv;

    std::vector<NEWMAT::SymmetricMatrix> _Mint;
    std::vector<NEWMAT::SymmetricMatrix> _Mintinv;

    std::vector<NEWMAT::DiagonalMatrix> _W;
    std::vector<NEWMAT::DiagonalMatrix> _Winv;

    std::vector<NEWMAT::DiagonalMatrix> _Regoa;
    std::vector<NEWMAT::DiagonalMatrix> _Regxy;
    std::vector<NEWMAT::DiagonalMatrix> _Regzrpy;
    std::vector<NEWMAT::DiagonalMatrix> _Regxyzrpy;
    std::vector<NEWMAT::DiagonalMatrix> _Regcog;
    std::vector<NEWMAT::DiagonalMatrix> _Regpc;
    std::vector<NEWMAT::DiagonalMatrix> _Regint;

    std::vector<NEWMAT::Matrix* > _JHP;
    std::vector<NEWMAT::Matrix* > _JHPplus;
    std::vector<NEWMAT::ColumnVector* > _dhp;

    std::vector<NEWMAT::ColumnVector> _balance_step;
    std::vector<NEWMAT::ColumnVector> _oa_step;
    std::vector<NEWMAT::ColumnVector> _zrpy_step;
    std::vector<NEWMAT::ColumnVector> _xy_step;
    std::vector<NEWMAT::ColumnVector> _xyzrpy_step;
    std::vector<NEWMAT::ColumnVector> _pc_step;
    std::vector<NEWMAT::ColumnVector> _int_step;
    std::vector<NEWMAT::ColumnVector> _step;

    dReal _cxy = 1.0;
    dReal _czrpy = 1.0;
    dReal _ccog = 0.2;
    dReal _coa = 1.0;
    dReal _cpc = 1.0;
    dReal _cint = 1.0;
    
    Transform _l_arm_manip_transform_offset;
    Transform _r_arm_manip_transform_offset;
    Transform _l_leg_manip_transform_offset;
    Transform _r_leg_manip_transform_offset;

    dReal epsilon = 0.005; //error tolerance for manipulator pose constraint
    std::vector<dReal> _xy_error;
    std::vector<dReal> _z_error;
    std::vector<dReal> _rpy_error;

    // bool bInCollision;
    // bool bLimit;
    bool bPrint;

    std::vector< std::vector<int> > _badjointinds;

    std::map<string,int> _GetManipIndex;

    std::vector< std::set<string> > _exclude_control_points;
    std::vector< std::set<string> > _links_in_collision;

    std::map<string,NEWMAT::Matrix> _giwc_database;

};


#endif
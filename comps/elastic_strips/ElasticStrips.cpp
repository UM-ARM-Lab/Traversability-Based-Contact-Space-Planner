/* Copyright (c) 2015 Worcester Polytechnic Institute
   Author: Yu-Chi Lin <ylin5@wpi.edu>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor Carnegie Mellon University,
       nor the names of their contributors, may be used to endorse or
       promote products derived from this software without specific prior
       written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file ElasticStrips.cpp

 *
 *
 * The Constrained BiDirectional Rapidly-exploring Random Tree (cbirrt) plugin contains the cbirrt problem, which parses commands from python or matlab and
 * calls the cbirrt planner. The cbirrt planner is a bidirectional sampling-based rrt which plans
 * on constraint manifolds induced by pose constraints as well as other constraints. This version of the software implements
 * pose constraints as Task Space Region (TSR) Chains. Pose constraints can apply to the start, goal, or entire path.
 * The planner is also set up to work with balance constraints.
 */
#include <omp.h>
#include <openrave/plugin.h>
#include "stdafx.h"

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    cout<<"Interface Create: "<<interfacename<<endl;

    if( type == PT_Module && interfacename == "elasticstrips" ) {
        return InterfaceBasePtr(new ElasticStrips(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("ElasticStrips");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}


void ElasticStrips::SetActiveRobots()
{

    if( _esRobotVec.size() == 0 )
    {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT(itrobot, _esRobotVec)
    {
        if( strcmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  )
        {
            _esRobot = *itrobot;
            break;
        }
    }

    if( _esRobot == NULL )
    {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}

int ElasticStrips::RunElasticStrips(ostream& sout, istream& sinput)
{
    // Lock environment mutex
    // EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    bPrint = false;
    // bPrint = true;

    boost::shared_ptr<ESParameters> params(new ESParameters());
    //params.reset(new ESParameters());

    RAVELOG_DEBUG("Starting Elastic Strips...\n");
    // RAVELOG_INFO("Checking ravelog_info\n");


    // RAVELOG_INFO("Initialize robot object.\n");
    // Initialize _esRobot

    sinput >> _strRobotName;
    // RAVELOG_INFO("Robot Name: %s\n",_strRobotName.c_str());

    GetEnv()->GetRobots(_esRobotVec);
    _num_robots = _esRobotVec.size();
    SetActiveRobots();

    _numdofs = _esRobot->GetActiveDOF();


    // Parameters to populate from sinput
    dReal temp_r;                      // Used in manip
    bool bGetTime = false;             // from gettime

    TransformMatrix temp_tf_matrix; // Used in maniptm
    Transform temp_tf;              // Used in maniptm

    Vector cogtarg(0, 0, 0);
    std::vector<string> supportlinks;   // from supportlinks
    Vector polyscale(1.0, 1.0, 1.0);    // from polyscale
    Vector polytrans(0, 0, 0);          // from polytrans
    std::vector<string> support_manips; // from support
    std::vector<dReal> support_mus;     // from support
    Vector gravity(0.0, 0.0, -9.8);     // from gravity

    int num_waypoints;
    dReal temp_q;

    // Populated from info in supportlinks
    std::vector<dReal> supportpolyx;
    std::vector<dReal> supportpolyy;

    // Store the trajectory
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(_esRobot->GetActiveConfigurationSpecification());


    if(bPrint)
        RAVELOG_INFO("Loading command\n");

    // Command string holds the current command
    string cmd;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        // RAVELOG_INFO(cmd);
        if(stricmp(cmd.c_str(), "nummanips") == 0) {
            // Number of manipulators whose transforms are specified.
            // Seems like this always agrees with the number of "maniptm" commands?
            sinput >> params->_num_manips;
        }
        else if(stricmp(cmd.c_str(), "manips") == 0) {
            for(int i = 0; i < params->_num_manips; i++)
            {
                sinput >> temp_r;
                params->_targmanips.push_back(temp_r);
            }
        }
        else if(stricmp(cmd.c_str(), "gettime") == 0) {
            // Whether the command's output should include the time.
            // If included it appears as the last space-separated number in the output string.
            bGetTime = true;
        }
        else if(stricmp(cmd.c_str(), "trajectory") == 0) {
            // This is mandatory for the elastic strips to work.
            sinput >> num_waypoints;
            std::vector<dReal> temp_waypoint(_numdofs);
            // int temp_index;
            for(int i = 0; i < num_waypoints; i++)
            {
                // sinput >> temp_index;
                for(int j = 0; j < _numdofs; j++)
                {
                    sinput >> temp_q;
                    temp_waypoint[j] = temp_q;
                }
                ptraj->Insert(ptraj->GetNumWaypoints(),temp_waypoint,_esRobot->GetActiveConfigurationSpecification());
            }
        }
        else if(stricmp(cmd.c_str(), "desiredmanippose") == 0) {
            // Specify the desired pose of each configuration in the trajectory.
            // Index, Number of specified pose in the configuration, The transformation.
            int num_desired_manip_pose;

            sinput >> num_desired_manip_pose;

            for(int i = 0; i < num_desired_manip_pose; i++)
            {
                int temp_index;
                int numspecifiedmanip;
                int temp_manip_index;
                dReal temp_contact_r;
                sinput >> temp_index;
                sinput >> numspecifiedmanip;

                std::map<string,std::pair<dReal,Transform> > temp_manip_pose;
                for(int j = 0; j < numspecifiedmanip; j++)
                {
                    sinput >> temp_manip_index;
                    sinput >> temp_contact_r;
                    sinput >> temp_tf_matrix.m[0];
                    sinput >> temp_tf_matrix.m[4];
                    sinput >> temp_tf_matrix.m[8];
                    sinput >> temp_tf_matrix.m[1];
                    sinput >> temp_tf_matrix.m[5];
                    sinput >> temp_tf_matrix.m[9];
                    sinput >> temp_tf_matrix.m[2];
                    sinput >> temp_tf_matrix.m[6];
                    sinput >> temp_tf_matrix.m[10];
                    sinput >> temp_tf_matrix.trans.x;
                    sinput >> temp_tf_matrix.trans.y;
                    sinput >> temp_tf_matrix.trans.z;
                    temp_tf = Transform(temp_tf_matrix);

                    temp_manip_pose.insert(std::pair<string,std::pair<dReal,Transform> >(_esRobot->GetManipulators()[temp_manip_index]->GetName(), std::make_pair(temp_contact_r,temp_tf)));
                }

                params->_desired_manip_pose.insert(std::pair<size_t, std::map<string,std::pair<dReal,Transform> > >(temp_index,temp_manip_pose));
            }

        }
        else if(stricmp(cmd.c_str(), "contact_manips") == 0){
            _waypoint_manip_group_map.clear();

            int temp_index;
            int contact_manip_num;
            string manip_name;
            int group_index;

            sinput >> num_waypoints;

            for(int i = 0; i < num_waypoints; i++)
            {
                sinput >> temp_index;
                sinput >> contact_manip_num;
                // std::vector<string> contact_manip;

                std::map<string,int> manip_group_map;

                for(int j = 0; j < contact_manip_num; j++)
                {
                    sinput >> manip_name;
                    sinput >> group_index;
                    manip_group_map.insert(std::pair<string,int>(manip_name,group_index));
                }

                _waypoint_manip_group_map.insert(std::pair< size_t, std::map<string,int> >(temp_index,manip_group_map));
            }

        }
        else if(stricmp(cmd.c_str(), "checkcollision") == 0) {
            params->bOBSTACLE_AVOIDANCE = true;
            int numcheckcollisionlinks;
            string tempstring;
            sinput >> numcheckcollisionlinks;
            for(int i = 0; i < numcheckcollisionlinks; i++)
            {
                sinput >> tempstring;
                params->_control_points.insert(std::pair<string,Vector>(tempstring, _esRobot->GetLink(tempstring)->GetCOMOffset()));
            }
        }
        else if(stricmp(cmd.c_str(), "checkselfcollision") == 0) {
            params->bOBSTACLE_AVOIDANCE = true;
            int numcheckselfcollisionlinkpairs;
            string tempstring_1;
            string tempstring_2;
            sinput >> numcheckselfcollisionlinkpairs;
            for(int i = 0; i < numcheckselfcollisionlinkpairs; i++)
            {
                sinput >> tempstring_1;
                sinput >> tempstring_2;
                params->_self_collision_checking_pairs.push_back(std::pair<string,string>(tempstring_1,tempstring_2));
            }
        }
        else if(stricmp(cmd.c_str(), "obstacles") == 0) {
            int numobstacles;
            string tempstring;
            sinput >> numobstacles;
            for(int i = 0; i < numobstacles; i++)
            {
                sinput >> tempstring;
                std::vector<KinBodyPtr> bodies;
                GetEnv()->GetBodies(bodies);


                Vector repulsive_vector;
                sinput >> repulsive_vector.x;
                sinput >> repulsive_vector.y;
                sinput >> repulsive_vector.z;

                for(unsigned int j = 0; j < bodies.size(); j++)
                {
                    if(stricmp(tempstring.c_str(),bodies[j]->GetName().c_str()) == 0)
                    {
                        params->_esObstacle.push_back( std::make_pair(bodies[j],repulsive_vector) );
                        break;
                    }
                }
            }
        }
        else if(stricmp(cmd.c_str(), "posturecontrol") == 0) {
            // define link relative transform respect to the robot base.
            // (@{LINK_NAME},${LINK_TRANSFORM})
            params->bPOSTURE_CONTROL = true;
            int numposturecontrol;
            string temp_link;
            sinput >> numposturecontrol;

            for(int i = 0; i < numposturecontrol; i++)
            {
                sinput >> temp_link;

                sinput >> temp_tf_matrix.m[0];
                sinput >> temp_tf_matrix.m[4];
                sinput >> temp_tf_matrix.m[8];
                sinput >> temp_tf_matrix.m[1];
                sinput >> temp_tf_matrix.m[5];
                sinput >> temp_tf_matrix.m[9];
                sinput >> temp_tf_matrix.m[2];
                sinput >> temp_tf_matrix.m[6];
                sinput >> temp_tf_matrix.m[10];
                sinput >> temp_tf_matrix.trans.x;
                sinput >> temp_tf_matrix.trans.y;
                sinput >> temp_tf_matrix.trans.z;
                temp_tf = Transform(temp_tf_matrix);

                params->_posture_control.insert(std::pair<string,Transform>(temp_link,temp_tf));

            }
        }
        else if(stricmp(cmd.c_str(), "supportlinks") == 0){
            // The links to use for support-polygon stability checking. Format is
            // "supportlinks {num_links} {link_name}..."
            params->balance_mode = BALANCE_SUPPORT_POLYGON;
            int numsupportlinks;
            string tempstring;
            sinput >> numsupportlinks;
            for(int i =0; i < numsupportlinks; i++)
            {
                sinput >> tempstring;
                supportlinks.push_back(tempstring);
            }
        }
        else if(stricmp(cmd.c_str(), "polyscale") == 0){
            // 3d vector to scale the support polygon for support-polygon stability checking. Intended to make it
            // possible to specify more conservative constraints (disallow barely-balanced situations).
            sinput >> polyscale.x;
            sinput >> polyscale.y;
            sinput >> polyscale.z;
        }
        else if(stricmp(cmd.c_str(), "polytrans") == 0){
            // 3d vector to translate the support polygon for support-polygon stability checking.
            sinput >> polytrans.x;
            sinput >> polytrans.y;
            sinput >> polytrans.z;
        }
        else if(stricmp(cmd.c_str(), "support") == 0){
            // Support specifier for GIWC stability checking. May be provided more than once. Format is
            // "support {manip_index} {friction_coeff}".
            string tempstr;
            sinput >> tempstr; support_manips.push_back(tempstr);
            sinput >> temp_r; support_mus.push_back(temp_r);
            params->balance_mode = BALANCE_GIWC;
        }
        else if(stricmp(cmd.c_str(), "gravity") == 0){
            sinput >> gravity.x;
            sinput >> gravity.y;
            sinput >> gravity.z;
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if(bPrint)
        RAVELOG_INFO("Commmand loaded\n");

    _balance_checkers.resize(num_waypoints);
    if(params->balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        if(bPrint)
            RAVELOG_INFO("Balance Mode: Support Polygon.\n");
        if(supportlinks.size() == 0)
        {
            RAVELOG_INFO("ERROR: Must specify support links to do balancing\n");
            return false;
        }

        std::map<string,int> link_name_index_map;
        for(int i = 0; i < supportlinks.size(); i++)
        {
            link_name_index_map.insert(std::make_pair(supportlinks[i],i));
        }

        for(std::map< size_t, std::map<string,int> >::iterator wmgm_it = _waypoint_manip_group_map.begin(); wmgm_it != _waypoint_manip_group_map.end(); wmgm_it++)
        {
            size_t waypoint_index = wmgm_it->first;
            std::map<string,int> manip_name_group_index_map = wmgm_it->second;

            std::vector<string> waypoint_support_links;

            for(std::map<string,int>::iterator mngim_it = manip_name_group_index_map.begin(); mngim_it != manip_name_group_index_map.end(); mngim_it++)
            {
                string manip_name = mngim_it->first;
                int manip_index = link_name_index_map[manip_name];
                waypoint_support_links.push_back(supportlinks[manip_index]);
            }

            std::shared_ptr<Balance> b = std::make_shared<Balance>(_esRobotVec[waypoint_index%_num_robots], waypoint_support_links, polyscale, polytrans);
            _balance_checkers[waypoint_index] = b;
        }

    }
    else if(params->balance_mode == BALANCE_GIWC)
    {
        if(bPrint)
            RAVELOG_INFO("Balance Mode: GIWC.\n");


        std::map<string,int> manip_name_index_map;
        for(int i = 0; i < support_manips.size(); i++)
        {
            manip_name_index_map.insert(std::make_pair(support_manips[i],i));
        }

        for(std::map< size_t, std::map<string,int> >::iterator wmgm_it = _waypoint_manip_group_map.begin(); wmgm_it != _waypoint_manip_group_map.end(); wmgm_it++)
        {
            size_t waypoint_index = wmgm_it->first;
            std::map<string,int> manip_name_group_index_map = wmgm_it->second;

            std::vector<dReal> waypoint_support_mus;
            std::vector<string> waypoint_support_manips;

            for(std::map<string,int>::iterator mngim_it = manip_name_group_index_map.begin(); mngim_it != manip_name_group_index_map.end(); mngim_it++)
            {
                string manip_name = mngim_it->first;
                int manip_index = manip_name_index_map[manip_name];
                waypoint_support_manips.push_back(support_manips[manip_index]);
                waypoint_support_mus.push_back(support_mus[manip_index]);
            }

            std::shared_ptr<Balance> b = std::make_shared<Balance>(_esRobotVec[waypoint_index%_num_robots], gravity, waypoint_support_manips, waypoint_support_mus);
            _balance_checkers[waypoint_index] = b;
        }

    }

    if(bPrint)
        RAVELOG_INFO("Balance checker initilize done.\n");

    unsigned long starttime = timeGetTime();

    if(bPrint)
        RAVELOG_INFO("Initialize planning variables.\n");

    InitPlan(params);

    if(bPrint)
        RAVELOG_INFO("Planning variables initialized. Now start planning.\n");

    std::vector<dReal> qResult(_numdofs);

    if(PlanPath(ptraj) == PS_HasSolution)
    {

        int timetaken = timeGetTime() - starttime;
        if(bPrint)
            RAVELOG_INFO("Num of waypoints: %i\n",ptraj->GetNumWaypoints());
        for(int w = 0; w < ptraj->GetNumWaypoints(); w++)
        {
            ptraj->GetWaypoint(w,qResult);
            sout<< w << " ";
            for(int i = 0; i < qResult.size(); i++)
            {
                sout << qResult[i] << " ";
            }
        }

        if(bGetTime)
            sout << timetaken << " ";

        RAVELOG_INFO("Elastic Strips Finished! (%d ms)\n",timetaken);
        return true;
    }
    else
    {
        int timetaken = timeGetTime() - starttime;

        if(bGetTime)
            sout << timetaken << " ";
        RAVELOG_INFO("Elastic Strips Failed to Converge. (%d ms)\n",timetaken);
        return true;
    }
}

Transform ElasticStrips::ForwardKinematics(std::vector<dReal> qs, RobotBasePtr robot, string manip_name)
{
    // EnvironmentMutex::scoped_lock lockenv(_esEnv->GetMutex());
    robot->SetActiveDOFValues(qs);
    return robot->GetManipulators()[_GetManipIndex.find(manip_name)->second]->GetTransform();
}

void ElasticStrips::LoadContactRegions()
{
    std::ifstream f_contact_region;
    f_contact_region.open("contact_regions.txt",std::ifstream::in);
    string data_string;
    _contact_regions.clear();
    while(getline(f_contact_region,data_string))
    {
        std::vector<float> data_vector;
        // Build an istream that holds the input string
        std::istringstream iss(data_string);

        // Iterate over the istream, using >> to grab floats
        // and push_back to store them in the vector
        std::copy(std::istream_iterator<float>(iss),std::istream_iterator<float>(),std::back_inserter(data_vector));

        Vector contact_position;
        Vector contact_normal;
        float contact_radius;

        contact_position.x = data_vector[0];
        contact_position.y = data_vector[1];
        contact_position.z = data_vector[2];

        contact_normal.x = data_vector[3];
        contact_normal.y = data_vector[4];
        contact_normal.z = data_vector[5];

        contact_radius = data_vector[6];

        _contact_regions.push_back(ContactRegion(contact_position,contact_normal,contact_radius));
    }
    f_contact_region.close();
}

void ElasticStrips::DecideContactConsistentTransform(TrajectoryBasePtr ptraj)
{
    for(std::map< int, ContactManipGroup >::iterator cmg_it = _contact_manips_group.begin(); cmg_it != _contact_manips_group.end(); cmg_it++)
    {

        std::vector<GraphHandlePtr> handles;
        int contact_manip_group_index = cmg_it->first;
        string manip_name = cmg_it->second.manip_name;
        Transform contact_consistent_transform = cmg_it->second.contact_consistent_transform;
        std::vector<size_t> wp_indices = cmg_it->second.waypoints;
        Vector axis_angle(0,0,0);
        Vector contact_consistent_translation(0,0,0);

        stringstream ss;
        ss << contact_manip_group_index;
        string contact_kinbody_name = "contact_" + ss.str();

        KinBodyPtr contact_kinbody = _esEnv->GetKinBody(contact_kinbody_name);

        // cout<<contact_kinbody_name<<": "<<manip_name<<", "<<contact_kinbody->GetTransform()<<endl;

        // cout<<"cct: "<<contact_consistent_transform.trans<<endl;

        NEWMAT::Matrix Q;
        Q.ReSize(4,wp_indices.size());

        for(std::vector<size_t>::iterator wi_it = wp_indices.begin(); wi_it != wp_indices.end(); wi_it++)
        {
            vector<dReal> qt;
            ptraj->GetWaypoint((*wi_it),qt);

            Transform manip_transform = ForwardKinematics(qt,_esRobot,manip_name);

            // axis_angle = axis_angle + (1.0/float(wp_indices.size())) * axisAngleFromQuat(manip_transform.rot);

            Q(1,wi_it-wp_indices.begin()+1) = manip_transform.rot.w;
            Q(2,wi_it-wp_indices.begin()+1) = manip_transform.rot.x;
            Q(3,wi_it-wp_indices.begin()+1) = manip_transform.rot.y;
            Q(4,wi_it-wp_indices.begin()+1) = manip_transform.rot.z;

            contact_consistent_translation = contact_consistent_translation + (1/float(wp_indices.size())) * manip_transform.trans;

            // cout<<manip_transform.trans<<endl;
            // Vector mt_origin = manip_transform.trans;
            // TransformMatrix manip_transform_matrix = TransformMatrix(manip_transform);
            // Vector mt_x = manip_transform.trans + 0.3 * Vector(manip_transform_matrix.m[0],manip_transform_matrix.m[4],manip_transform_matrix.m[8]);
            // Vector mt_y = manip_transform.trans + 0.3 * Vector(manip_transform_matrix.m[1],manip_transform_matrix.m[5],manip_transform_matrix.m[9]);
            // Vector mt_z = manip_transform.trans + 0.3 * Vector(manip_transform_matrix.m[2],manip_transform_matrix.m[6],manip_transform_matrix.m[10]);

            // handles.push_back(_esEnv->drawarrow(mt_origin, mt_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
            // handles.push_back(_esEnv->drawarrow(mt_origin, mt_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
            // handles.push_back(_esEnv->drawarrow(mt_origin, mt_z, 0.008, RaveVector<float>(0, 0, 1, 1)));
        }

        NEWMAT::SymmetricMatrix QQ;
        QQ.ReSize(4);
        QQ << Q * Q.t();

        NEWMAT::DiagonalMatrix EigenValue;
        EigenValue.ReSize(4);
        NEWMAT::Matrix EigenVector;
        EigenVector.ReSize(4,4);

        NEWMAT::EigenValues(QQ, EigenValue, EigenVector);

        contact_consistent_transform.rot.w = EigenVector(1,4);
        contact_consistent_transform.rot.x = EigenVector(2,4);
        contact_consistent_transform.rot.y = EigenVector(3,4);
        contact_consistent_transform.rot.z = EigenVector(4,4);

        // contact_consistent_transform.rot = quatFromAxisAngle(axis_angle);

        // if(strcmp(manip_name.c_str(),"l_leg") == 0 || strcmp(manip_name.c_str(),"r_leg") == 0)
        // {
        //     axis_angle = axisAngleFromQuat(contact_consistent_transform.rot);
        //     axis_angle[2] = min(max(axis_angle[2],-20*PI/180.0),20*PI/180.0);
        //     axis_angle[2] = 0;
        //     contact_consistent_transform.rot = quatFromAxisAngle(axis_angle);
        // }


        contact_consistent_transform.trans = contact_consistent_translation;

        if(contact_kinbody)
        {
            contact_kinbody->SetTransform(contact_consistent_transform);
        }

        // cout<<contact_kinbody_name<<"Type: "<<manip_name.c_str()<<", Transform: "<<contact_kinbody->GetTransform()<<endl;
        // getchar();

        if(strcmp(manip_name.c_str(),"l_arm") == 0)
        {
            contact_consistent_transform = contact_consistent_transform * _l_arm_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"r_arm") == 0)
        {
            contact_consistent_transform = contact_consistent_transform * _r_arm_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"l_leg") == 0)
        {
            contact_consistent_transform = contact_consistent_transform * _l_leg_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"r_leg") == 0)
        {
            contact_consistent_transform = contact_consistent_transform * _r_leg_manip_transform_offset;
        }

        cmg_it->second.contact_consistent_transform = contact_consistent_transform;

        // cout<<contact_kinbody_name<<": "<<contact_kinbody->GetTransform()<<endl;

        // Vector cct_origin = contact_consistent_transform.trans;
        // TransformMatrix contact_consistent_transform_matrix = TransformMatrix(contact_consistent_transform);
        // Vector cct_x = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[0],contact_consistent_transform_matrix.m[4],contact_consistent_transform_matrix.m[8]);
        // Vector cct_y = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[1],contact_consistent_transform_matrix.m[5],contact_consistent_transform_matrix.m[9]);
        // Vector cct_z = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[2],contact_consistent_transform_matrix.m[6],contact_consistent_transform_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

        // getchar();

    }
}

int ElasticStrips::FindNearestContactRegion()
{
    std::map< int, ContactRegion > temp_nearest_contact_regions;
    std::vector<GraphHandlePtr> handles;
    float contact_r = 0.142 * 1.5;

    Transform last_left_foot_xy_transform, last_right_foot_xy_transform;
    Transform standing_foot_xy_transform, standing_foot_xy_transform_inverse;

    bool first_foot_contact = true;

    bool first_left_foot_contact = true;
    bool first_right_foot_contact = true;

    std::vector<std::pair<int,ContactManipGroup> > contact_manips_group_vec;

    int index = 0;
    int counter = 0;

    while(counter < _contact_manips_group.size())
    {
        if(_contact_manips_group.count(index) != 0)
        {
            std::map< int, ContactManipGroup >::iterator cmg_it = _contact_manips_group.find(index);
            contact_manips_group_vec.push_back(std::make_pair(cmg_it->first,cmg_it->second));
            counter++;
        }
        index++;
    }

    for(std::vector< std::pair< int, ContactManipGroup > >::iterator cmg_it = contact_manips_group_vec.begin(); cmg_it != contact_manips_group_vec.end(); cmg_it++)
    {
        int contact_manip_group_index = cmg_it->first;
        string manip_name = cmg_it->second.manip_name;
        Transform contact_consistent_transform = cmg_it->second.contact_consistent_transform;

        std::vector<ContactRegion> candidate_contact_regions;

        // Fix the starting and ending footsteps at the start and goal region
        // if(contact_manip_group_index == start_contact_group_index[0] || contact_manip_group_index == start_contact_group_index[1])
        // {
        //     candidate_contact_regions.push_back(_contact_regions.at(_contact_regions.size() - 2));
        // }
        // else if(contact_manip_group_index == goal_contact_group_index[0] || contact_manip_group_index == goal_contact_group_index[1])
        // {
        //     candidate_contact_regions.push_back(_contact_regions.at(_contact_regions.size() - 1));
        // }
        // else
        // {
        //     candidate_contact_regions = _contact_regions;
        // }
        candidate_contact_regions = _contact_regions;

        // cout<<candidate_contact_regions.size()<<endl;

        bool is_foot_contact = false;
        bool is_right_leg = false;

        if(strcmp(manip_name.c_str(),"l_leg") == 0)
        {
            standing_foot_xy_transform = last_right_foot_xy_transform;
            standing_foot_xy_transform_inverse = last_right_foot_xy_transform.inverse();
            is_foot_contact = true;
        }
        else if(strcmp(manip_name.c_str(),"r_leg") == 0)
        {
            standing_foot_xy_transform = last_left_foot_xy_transform;
            standing_foot_xy_transform_inverse = last_left_foot_xy_transform.inverse();
            is_foot_contact = true;
            is_right_leg = true;
        }

        bool is_hand_contact = !is_foot_contact;

        bool found_contact_region = false;

        float nearest_dist = 3.0;
        Transform final_contact_consistent_transform;
        ContactRegion nearest_contact_region;

        for(std::vector<ContactRegion>::iterator cr_it = candidate_contact_regions.begin(); cr_it != candidate_contact_regions.end(); cr_it++)
        {
            if(is_foot_contact && fabs(cr_it->normal.z) < 0.85)
            {
                continue;
            }
            else if(is_hand_contact && fabs(cr_it->normal.z) >= 0.85)
            {
                continue;
            }

            // if(is_foot_contact && cr_it->contact_region_frame.trans.z > 0.4)
            // {
            //     continue;
            // }
            // else if(is_hand_contact && cr_it->contact_region_frame.trans.z <= 0.4)
            // {
            //     continue;
            // }

            float dist = cr_it->DistToContactRegion(manip_name,contact_consistent_transform);

            if(dist < nearest_dist)
            {
                // check if it will be in collision with previous contact groups
                Transform temp_contact_region_frame = cr_it->contact_region_frame;
                Transform temp_contact_region_frame_inverse = temp_contact_region_frame.inverse();

                Transform temp_projected_contact_consistent_transform = temp_contact_region_frame_inverse * contact_consistent_transform;



                temp_projected_contact_consistent_transform.trans.z = 0.0;

                float dist_to_center = sqrt(temp_projected_contact_consistent_transform.trans.lengthsqr2());

                if(dist_to_center > cr_it->radius)
                {
                    temp_projected_contact_consistent_transform.trans.x *= (cr_it->radius/dist_to_center);
                    temp_projected_contact_consistent_transform.trans.y *= (cr_it->radius/dist_to_center);
                }

                Vector temp_projected_contact_consistent_transform_axis_angle = axisAngleFromQuat(temp_projected_contact_consistent_transform.rot);
                temp_projected_contact_consistent_transform_axis_angle.x = 0.0;
                temp_projected_contact_consistent_transform_axis_angle.y = 0.0;

                temp_projected_contact_consistent_transform.rot = quatFromAxisAngle(temp_projected_contact_consistent_transform_axis_angle);

                Transform new_contact_consistent_transform = temp_contact_region_frame * temp_projected_contact_consistent_transform;

                // update the foot contact consistent pose based on the limit area
                if(is_foot_contact && !first_foot_contact)
                {
                    Transform standing_foot_to_new_contact_consistent_transform = standing_foot_xy_transform_inverse * new_contact_consistent_transform;
                    dReal tx = standing_foot_to_new_contact_consistent_transform.trans.x;
                    dReal ty = standing_foot_to_new_contact_consistent_transform.trans.y;

                    if(is_right_leg)
                    {
                        ty = -ty;
                    }

                    bool in_left_bound = (pow(tx/0.45,2) + pow((ty-0.2)/0.3,2) <= 1);
                    bool in_right_bound = (ty >= 0.2);
                    in_left_bound = true;

                    if(in_left_bound && in_right_bound)
                    {
                        found_contact_region = true;
                    }
                    else
                    {
                        if(in_left_bound && !in_right_bound)
                        {
                            ty = 0.2;
                        }
                        else if(!in_left_bound && in_right_bound)
                        {
                            dReal m = 1.0/sqrt(pow(tx/0.45,2) + pow((ty-0.2)/0.3,2));
                            tx = m*tx;
                            ty = 0.2 + m*(ty-0.2);
                        }
                        else
                        {
                            if(tx >= 0.45)
                            {
                                tx = 0.45;
                            }
                            else if(tx <= -0.45)
                            {
                                tx = -0.45;
                            }
                            ty = 0.2;
                        }

                        if(is_right_leg)
                        {
                            ty = -ty;
                        }

                        // transform the new projected distance back and see if it is inside the original contact region
                        standing_foot_to_new_contact_consistent_transform.trans.x = tx;
                        standing_foot_to_new_contact_consistent_transform.trans.y = ty;
                        temp_projected_contact_consistent_transform = temp_contact_region_frame_inverse * standing_foot_xy_transform * standing_foot_to_new_contact_consistent_transform;

                        temp_projected_contact_consistent_transform.trans.z = 0.0;
                        dist_to_center = sqrt(temp_projected_contact_consistent_transform.trans.lengthsqr2());

                        if(dist_to_center <= cr_it->radius)
                        {
                            found_contact_region = true;
                            new_contact_consistent_transform = temp_contact_region_frame * temp_projected_contact_consistent_transform;
                            float new_dist = cr_it->DistToContactRegion(manip_name,contact_consistent_transform,new_contact_consistent_transform.trans);

                            if(new_dist < dist-0.0001)
                            {
                                cout << "Bug!!!" << dist << " " << new_dist << endl;
                                getchar();
                            }
                            dist = new_dist;
                        }
                        else
                        {
                            dist = 9999.0;
                        }
                    }
                }
                // update contact consisten pose of the hand contacts
                else if(is_hand_contact && !first_left_foot_contact && !first_right_foot_contact)
                {
                    Vector feet_mean_position = 0.5 * (last_left_foot_xy_transform.trans + last_right_foot_xy_transform.trans);

                    bool inside_armreach_cylinder = sqrt((new_contact_consistent_transform.trans-feet_mean_position).lengthsqr2()) <= 0.9;
                    bool inside_contact_region = sqrt((new_contact_consistent_transform.trans - cr_it->position).lengthsqr3()) <= cr_it->radius;
                    bool region_within_reach = sqrt((cr_it->position-feet_mean_position).lengthsqr2()) <= 1.0 + cr_it->radius;

                    if(!inside_armreach_cylinder && region_within_reach)
                    {
                        Vector arc_vector = cr_it->normal.cross(Vector(0,0,1));
                        Vector arc_unit_vector = arc_vector.normalize3();
                        Vector step_vector;
                        Vector step_unit_vector;

                        float step_size = 0.01;

                        do
                        {
                            step_vector = (feet_mean_position-new_contact_consistent_transform.trans).dot3(arc_unit_vector) * arc_unit_vector;

                            if(sqrt(step_vector.lengthsqr2()) < 0.01)
                            {
                                break;
                            }

                            step_unit_vector = step_vector.normalize3();

                            new_contact_consistent_transform.trans = new_contact_consistent_transform.trans + step_size * step_unit_vector;

                            inside_armreach_cylinder = sqrt((new_contact_consistent_transform.trans-feet_mean_position).lengthsqr2()) <= 0.9;

                            inside_contact_region = sqrt((new_contact_consistent_transform.trans - cr_it->position).lengthsqr3()) <= cr_it->radius;

                        }while(!inside_armreach_cylinder && inside_contact_region);

                        if(inside_armreach_cylinder)
                        {
                            found_contact_region = true;
                            dist = cr_it->DistToContactRegion(manip_name,contact_consistent_transform,new_contact_consistent_transform.trans);
                        }
                        else
                        {
                            dist = 9999.0;
                        }
                    }
                    else
                    {
                        found_contact_region = true;
                    }

                }
                else
                {
                    found_contact_region = true;
                }

                if(dist < nearest_dist)
                {
                    nearest_dist = dist;
                    nearest_contact_region = *cr_it;
                    final_contact_consistent_transform = new_contact_consistent_transform;
                }
            }

            if(nearest_dist == 0.0)
            {
                break;
            }
        }

        if(!found_contact_region)
        {
            return -1;
        }

        _contact_manips_group.find(contact_manip_group_index)->second.contact_consistent_transform = final_contact_consistent_transform;

        Transform contact_region_frame = nearest_contact_region.contact_region_frame;

        temp_nearest_contact_regions.insert(std::pair<int,ContactRegion>(contact_manip_group_index,nearest_contact_region));

        // get the new left or right leg xy transform
        if(strcmp(manip_name.c_str(),"l_leg") == 0)
        {
            // modify the contact consisten transform to be it xy transform
            RaveTransformMatrix<dReal> final_contact_consistent_transform_matrix = RaveTransformMatrix<dReal>(final_contact_consistent_transform);
            dReal yaw = atan2(final_contact_consistent_transform_matrix.m[4], final_contact_consistent_transform_matrix.m[0]);
            last_left_foot_xy_transform.trans = final_contact_consistent_transform.trans;
            last_left_foot_xy_transform.rot = quatFromAxisAngle(Vector(0,0,yaw));
            first_left_foot_contact = false;
            first_foot_contact = false;
        }
        else if(strcmp(manip_name.c_str(),"r_leg") == 0)
        {
            // modify the contact consisten transform to be it xy transform
            RaveTransformMatrix<dReal> final_contact_consistent_transform_matrix = RaveTransformMatrix<dReal>(final_contact_consistent_transform);
            dReal yaw = atan2(final_contact_consistent_transform_matrix.m[4], final_contact_consistent_transform_matrix.m[0]);
            last_right_foot_xy_transform.trans = final_contact_consistent_transform.trans;
            last_right_foot_xy_transform.rot = quatFromAxisAngle(Vector(0,0,yaw));
            first_right_foot_contact = false;
            first_foot_contact = false;
        }


        // TransformMatrix contact_region_frame_matrix = TransformMatrix(contact_region_frame);
        // Vector crf_origin = contact_region_frame.trans;
        // Vector crf_x = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[0],contact_region_frame_matrix.m[4],contact_region_frame_matrix.m[8]);
        // Vector crf_y = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[1],contact_region_frame_matrix.m[5],contact_region_frame_matrix.m[9]);
        // Vector crf_z = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[2],contact_region_frame_matrix.m[6],contact_region_frame_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(crf_origin, crf_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(crf_origin, crf_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(crf_origin, crf_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

        // getchar();

        // TransformMatrix contact_consistent_transform_matrix = TransformMatrix(contact_consistent_transform);
        // Vector cct_origin = contact_consistent_transform.trans;
        // Vector cct_x = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[0],contact_consistent_transform_matrix.m[4],contact_consistent_transform_matrix.m[8]);
        // Vector cct_y = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[1],contact_consistent_transform_matrix.m[5],contact_consistent_transform_matrix.m[9]);
        // Vector cct_z = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[2],contact_consistent_transform_matrix.m[6],contact_consistent_transform_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

        // cout<<"Contact consistent transform"<<endl;
        // getchar();

        // TransformMatrix final_contact_consistent_transform_matrix = TransformMatrix(final_contact_consistent_transform);
        // Vector fcct_origin = final_contact_consistent_transform.trans;
        // Vector fcct_x = final_contact_consistent_transform.trans + 0.5 * Vector(final_contact_consistent_transform_matrix.m[0],final_contact_consistent_transform_matrix.m[4],final_contact_consistent_transform_matrix.m[8]);
        // Vector fcct_y = final_contact_consistent_transform.trans + 0.5 * Vector(final_contact_consistent_transform_matrix.m[1],final_contact_consistent_transform_matrix.m[5],final_contact_consistent_transform_matrix.m[9]);
        // Vector fcct_z = final_contact_consistent_transform.trans + 0.5 * Vector(final_contact_consistent_transform_matrix.m[2],final_contact_consistent_transform_matrix.m[6],final_contact_consistent_transform_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(fcct_origin, fcct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(fcct_origin, fcct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(fcct_origin, fcct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

        // cout<<"New contact consistent transform"<<endl;
        // getchar();

    }


    _nearest_contact_regions = temp_nearest_contact_regions;
    return 1;

}

// void ElasticStrips::FindNearestContactRegion()
// {
//     std::map< int, ContactRegion > temp_nearest_contact_regions;
//     std::vector<GraphHandlePtr> handles;

//     for(std::map< int, ContactManipGroup >::iterator cmg_it = contact_manips_group.begin(); cmg_it != contact_manips_group.end(); cmg_it++)
//     {
//         int contact_manip_group_index = cmg_it->first;
//         string manip_name = cmg_it->second.manip_name;
//         Transform contact_consistent_transform = cmg_it->second.contact_consistent_transform;
//         ContactRegion nearest_contact_region;


//         if(contact_manip_group_index == start_contact_group_index[0] || contact_manip_group_index == start_contact_group_index[1])
//         {
//             nearest_contact_region = _contact_regions.at(_contact_regions.size() - 2);
//         }
//         else if(contact_manip_group_index == goal_contact_group_index[0] || contact_manip_group_index == goal_contact_group_index[1])
//         {
//             nearest_contact_region = _contact_regions.at(_contact_regions.size() - 1);
//         }
//         else
//         {
//             float nearest_dist = 9999.0;

//             for(std::vector<ContactRegion>::iterator cr_it = _contact_regions.begin(); cr_it != _contact_regions.end(); cr_it++)
//             {
//                 float dist = cr_it->DistToContactRegion(manip_name,contact_consistent_transform);

//                 if(dist < nearest_dist)
//                 {
//                     nearest_dist = dist;
//                     nearest_contact_region = *cr_it;
//                 }

//                 if(nearest_dist == 0.0)
//                 {
//                     break;
//                 }
//             }
//         }

//         temp_nearest_contact_regions.insert(std::pair<int,ContactRegion>(contact_manip_group_index,nearest_contact_region));

//         // project transform to the contact region

//         Transform contact_region_frame = nearest_contact_region.contact_region_frame;

//         Transform projected_contact_consistent_transform = contact_region_frame.inverse() * contact_consistent_transform;

//         projected_contact_consistent_transform.trans.z = 0.0;

//         float dist_to_center = sqrt(projected_contact_consistent_transform.trans.lengthsqr3());


//         if(dist_to_center > nearest_contact_region.radius)
//         {
//             projected_contact_consistent_transform.trans.x *= (nearest_contact_region.radius/dist_to_center);
//             projected_contact_consistent_transform.trans.y *= (nearest_contact_region.radius/dist_to_center);
//         }

//         Vector projected_contact_consistent_transform_axis_angle = axisAngleFromQuat(projected_contact_consistent_transform.rot);
//         projected_contact_consistent_transform_axis_angle.x = 0.0;
//         projected_contact_consistent_transform_axis_angle.y = 0.0;


//         projected_contact_consistent_transform.rot = quatFromAxisAngle(projected_contact_consistent_transform_axis_angle);

//         cmg_it->second.contact_consistent_transform = contact_region_frame * projected_contact_consistent_transform;

//         Transform new_contact_consistent_transform = contact_region_frame * projected_contact_consistent_transform;

//         TransformMatrix contact_region_frame_matrix = TransformMatrix(contact_region_frame);

//         // Vector crf_origin = contact_region_frame.trans;
//         // Vector crf_x = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[0],contact_region_frame_matrix.m[4],contact_region_frame_matrix.m[8]);
//         // Vector crf_y = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[1],contact_region_frame_matrix.m[5],contact_region_frame_matrix.m[9]);
//         // Vector crf_z = contact_region_frame.trans + 0.3 * Vector(contact_region_frame_matrix.m[2],contact_region_frame_matrix.m[6],contact_region_frame_matrix.m[10]);

//         // handles.push_back(_esEnv->drawarrow(crf_origin, crf_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
//         // handles.push_back(_esEnv->drawarrow(crf_origin, crf_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
//         // handles.push_back(_esEnv->drawarrow(crf_origin, crf_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

//         // getchar();

//         Vector cct_origin = contact_consistent_transform.trans;
//         TransformMatrix contact_consistent_transform_matrix = TransformMatrix(contact_consistent_transform);
//         Vector cct_x = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[0],contact_consistent_transform_matrix.m[4],contact_consistent_transform_matrix.m[8]);
//         Vector cct_y = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[1],contact_consistent_transform_matrix.m[5],contact_consistent_transform_matrix.m[9]);
//         Vector cct_z = contact_consistent_transform.trans + 0.4 * Vector(contact_consistent_transform_matrix.m[2],contact_consistent_transform_matrix.m[6],contact_consistent_transform_matrix.m[10]);

//         handles.push_back(_esEnv->drawarrow(cct_origin, cct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
//         handles.push_back(_esEnv->drawarrow(cct_origin, cct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
//         handles.push_back(_esEnv->drawarrow(cct_origin, cct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

//         // cout<<"Contact consistent transform"<<endl;
//         // getchar();

//         Vector ncct_origin = new_contact_consistent_transform.trans;
//         TransformMatrix new_contact_consistent_transform_matrix = TransformMatrix(new_contact_consistent_transform);
//         Vector ncct_x = new_contact_consistent_transform.trans + 0.5 * Vector(new_contact_consistent_transform_matrix.m[0],new_contact_consistent_transform_matrix.m[4],new_contact_consistent_transform_matrix.m[8]);
//         Vector ncct_y = new_contact_consistent_transform.trans + 0.5 * Vector(new_contact_consistent_transform_matrix.m[1],new_contact_consistent_transform_matrix.m[5],new_contact_consistent_transform_matrix.m[9]);
//         Vector ncct_z = new_contact_consistent_transform.trans + 0.5 * Vector(new_contact_consistent_transform_matrix.m[2],new_contact_consistent_transform_matrix.m[6],new_contact_consistent_transform_matrix.m[10]);

//         handles.push_back(_esEnv->drawarrow(ncct_origin, ncct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
//         handles.push_back(_esEnv->drawarrow(ncct_origin, ncct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
//         handles.push_back(_esEnv->drawarrow(ncct_origin, ncct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

//         // cout<<"New contact consistent transform"<<endl;
//         // getchar();

//     }


//     nearest_contact_regions = temp_nearest_contact_regions;

// }

void ElasticStrips::FindContactRegions()
{
    _contact_consistent_region.clear();
    for(map<size_t, map<string,std::pair<dReal,Transform> > >::iterator dmp_it = _parameters->_desired_manip_pose.begin(); dmp_it != _parameters->_desired_manip_pose.end(); dmp_it++) // all contact regions
    {
        map<string,int> temp_contact_region;
        for(map<string,std::pair<dReal,Transform> >::iterator p_it = dmp_it->second.begin(); p_it != dmp_it->second.end(); p_it++) // contact regions in the same waypoint
        {
            bool matched_contact_region_found = false;
            Vector desired_trans = p_it->second.second.trans;
            string desired_manip_name = p_it->first;
            // cout<<"desired manip name: "<<desired_manip_name<<", desired_trans: ("<<desired_trans.x<<","<<desired_trans.y<<","<<desired_trans.z<<")"<<endl;
            for(std::vector< std::pair<string,Vector> >::iterator ccmt_it = _contact_consistent_manip_translation.begin(); ccmt_it != _contact_consistent_manip_translation.end(); ccmt_it++)
            {
                string temp_manip_name = ccmt_it->first;
                Vector temp_contact_manip_trans = ccmt_it->second;
                if(temp_manip_name == desired_manip_name)
                {
                    if(fabs(desired_trans.x - temp_contact_manip_trans.x)+
                       fabs(desired_trans.y - temp_contact_manip_trans.y)+
                       fabs(desired_trans.z - temp_contact_manip_trans.z) < 0.001)
                    {
                        // cout<<"manipulator: "<<temp_manip_name<<", desired_trans: ("<<desired_trans.x<<","<<desired_trans.y<<","<<desired_trans.z<<"), temp_contact_manip_trans: ("<<temp_contact_manip_trans.x<<","<<temp_contact_manip_trans.y<<","<<temp_contact_manip_trans.z<<")"<<endl;
                        map<size_t, map<string,std::pair<dReal,Transform> > >::iterator prev_dmp_it = dmp_it;
                        prev_dmp_it--;
                        int contact_region_index = ccmt_it - _contact_consistent_manip_translation.begin();
                        if(_contact_consistent_region[prev_dmp_it->first].count(desired_manip_name) == 0)
                        {
                            // cout<<"manipulator "<<desired_manip_name<<" is not in contact in the last waypoint"<<endl;
                            break;
                        }
                        int p_contact_region_index = _contact_consistent_region[prev_dmp_it->first].find(desired_manip_name)->second;
                        // if the contact region index now is the same as the contact region index in the last waypoint with the same manip, it is clustered to the same contact.
                        // cout<<"contact region index: "<<contact_region_index<<", previous contact region index: "<<p_contact_region_index<<endl;
                        if(contact_region_index == p_contact_region_index)
                        {
                            // cout<<"matched contact region found."<<endl;
                            temp_contact_region.insert(std::pair<string,int>(desired_manip_name, contact_region_index));
                            matched_contact_region_found = true;
                            break;
                        }
                    }
                }
            }

            if(!matched_contact_region_found)
            {
                // cout<<"no matched contact region. add a new entry."<<endl;
                temp_contact_region.insert(std::pair<string,int>(desired_manip_name, _contact_consistent_manip_translation.size()));
                _contact_consistent_manip_translation.push_back(std::pair<string,Vector>(desired_manip_name,desired_trans));
            }
            // cout<<"contact_consistent_manip_translation size: "<<contact_consistent_manip_translation.size()<<endl;
        }

        _contact_consistent_region.push_back(temp_contact_region);
    }
}

void ElasticStrips::FindContactConsistentManipTranslation(TrajectoryBasePtr ptraj)
{
    std::vector< std::vector<Vector> > temp_contact_consistent_manip_translations(_contact_consistent_manip_translation.size());
    for(unsigned int w = 0; w < ptraj->GetNumWaypoints(); w++)
    {
        vector<dReal> qt;
        ptraj->GetWaypoint(w,qt);

        std::map<string,int> contact_regions = _contact_consistent_region[w];

        for(std::map<string,int>::iterator cr_it = contact_regions.begin(); cr_it != contact_regions.end(); cr_it++)
        {
            string temp_manip_name = cr_it->first;
            int temp_region_index = cr_it->second;
            Transform local_desired_pose_frame = _parameters->_desired_manip_pose.find(w)->second.find(temp_manip_name)->second.second;
            dReal contact_region_radius = _parameters->_desired_manip_pose.find(w)->second.find(temp_manip_name)->second.first;
            Vector contact_position = ForwardKinematics(qt,_esRobot,temp_manip_name).trans;
            // cout<<"local_desired_pose_frame: "<<local_desired_pose_frame<<endl;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position = local_desired_pose_frame.inverse() * contact_position;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position.z = 0;
            dReal dist_to_contact_region_center = sqrt(contact_position.x*contact_position.x + contact_position.y*contact_position.y);
            if(dist_to_contact_region_center > contact_region_radius)
            {
                contact_position.x *= (contact_region_radius/dist_to_contact_region_center);
                contact_position.y *= (contact_region_radius/dist_to_contact_region_center);
            }
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            contact_position = local_desired_pose_frame * contact_position;
            // cout<<"contact_position: "<<contact_position.x<<" "<<contact_position.y<<" "<<contact_position.z<<endl;
            temp_contact_consistent_manip_translations.at(temp_region_index).push_back(contact_position);
        }
    }

    for(std::vector< std::vector<Vector> >::iterator tccmt_it = temp_contact_consistent_manip_translations.begin(); tccmt_it != temp_contact_consistent_manip_translations.end(); tccmt_it++)
    {
        Vector v(0,0,0);
        int contact_region_index = tccmt_it - temp_contact_consistent_manip_translations.begin();
        for(std::vector<Vector>::iterator it = tccmt_it->begin(); it != tccmt_it->end(); it++)
        {
            v.x += it->x;
            v.y += it->y;
            v.z += it->z;
        }
        v.x /= tccmt_it->size();
        v.y /= tccmt_it->size();
        v.z /= tccmt_it->size();

        if(tccmt_it->size() == 0)
            cout<<"temp_contact_consistent_manip_translation.size == 0, bug!!!"<<endl;
            getchar();

        _contact_consistent_manip_translation.at(contact_region_index).second = v;
    }
}


void ElasticStrips::InitPlan(boost::shared_ptr<ESParameters> params)
{
    //0. parameter initialization
    _esRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);

    for(int i = 0; i < _esRobot->GetManipulators().size(); i++)
    {
        _GetManipIndex.insert(std::pair<string,int>(_esRobot->GetManipulators()[i]->GetName(),i));
    }

    //1. receive command
    // RobotBase::ManipulatorConstPtr activemanip_p = _esRobot->GetActiveManipulator();

    // dReal* qResult = &(*result.get())[0];
    // const dReal* pFreeParameters = &vFreeParameters[0];
    // solutionpath.resize(0);
    // // _targmanips.resize(0);
    // // _targtms.resize(0);
    // movementlimit = INF;

    // bool bsuccess = true;

    _parameters = params;

    _parameters->_internal_force_links.clear();
    _parameters->_internal_force_links.push_back("l_foot");
    _parameters->_internal_force_links.push_back("r_foot");
    _parameters->_internal_force_links.push_back("l_palm");
    _parameters->_internal_force_links.push_back("r_palm");

    //Group the contact regions
    // FindContactRegions();

    //Load all contact regions
    LoadContactRegions();

    TransformMatrix l_arm_manip_transform_offset_matrix;
    l_arm_manip_transform_offset_matrix.trans = Vector(0,0,0);
    l_arm_manip_transform_offset_matrix.m[0] = 0; l_arm_manip_transform_offset_matrix.m[1] = 0; l_arm_manip_transform_offset_matrix.m[2] = -1;
    l_arm_manip_transform_offset_matrix.m[4] = 1; l_arm_manip_transform_offset_matrix.m[5] = 0; l_arm_manip_transform_offset_matrix.m[6] = 0;
    l_arm_manip_transform_offset_matrix.m[8] = 0; l_arm_manip_transform_offset_matrix.m[9] = -1; l_arm_manip_transform_offset_matrix.m[10] = 0;
    _l_arm_manip_transform_offset = Transform(l_arm_manip_transform_offset_matrix);

    TransformMatrix r_arm_manip_transform_offset_matrix;
    r_arm_manip_transform_offset_matrix.trans = Vector(0,0,0);
    r_arm_manip_transform_offset_matrix.m[0] = 0; r_arm_manip_transform_offset_matrix.m[1] = 0; r_arm_manip_transform_offset_matrix.m[2] = -1;
    r_arm_manip_transform_offset_matrix.m[4] = -1; r_arm_manip_transform_offset_matrix.m[5] = 0; r_arm_manip_transform_offset_matrix.m[6] = 0;
    r_arm_manip_transform_offset_matrix.m[8] = 0; r_arm_manip_transform_offset_matrix.m[9] = 1; r_arm_manip_transform_offset_matrix.m[10] = 0;
    _r_arm_manip_transform_offset = Transform(r_arm_manip_transform_offset_matrix);

    TransformMatrix l_leg_manip_transform_offset_matrix;
    l_leg_manip_transform_offset_matrix.trans = Vector(0,0,0);
    l_leg_manip_transform_offset_matrix.m[0] = 1; l_leg_manip_transform_offset_matrix.m[1] = 0; l_leg_manip_transform_offset_matrix.m[2] = 0;
    l_leg_manip_transform_offset_matrix.m[4] = 0; l_leg_manip_transform_offset_matrix.m[5] = 1; l_leg_manip_transform_offset_matrix.m[6] = 0;
    l_leg_manip_transform_offset_matrix.m[8] = 0; l_leg_manip_transform_offset_matrix.m[9] = 0; l_leg_manip_transform_offset_matrix.m[10] = 1;
    _l_leg_manip_transform_offset = Transform(l_leg_manip_transform_offset_matrix);

    TransformMatrix r_leg_manip_transform_offset_matrix;
    r_leg_manip_transform_offset_matrix.trans = Vector(0,0,0);
    r_leg_manip_transform_offset_matrix.m[0] = 1; r_leg_manip_transform_offset_matrix.m[1] = 0; r_leg_manip_transform_offset_matrix.m[2] = 0;
    r_leg_manip_transform_offset_matrix.m[4] = 0; r_leg_manip_transform_offset_matrix.m[5] = 1; r_leg_manip_transform_offset_matrix.m[6] = 0;
    r_leg_manip_transform_offset_matrix.m[8] = 0; r_leg_manip_transform_offset_matrix.m[9] = 0; r_leg_manip_transform_offset_matrix.m[10] = 1;
    _r_leg_manip_transform_offset = Transform(r_leg_manip_transform_offset_matrix);


    for(std::map< size_t, std::map<string,int> >::iterator wmgm_it = _waypoint_manip_group_map.begin(); wmgm_it != _waypoint_manip_group_map.end(); wmgm_it++)
    {
        size_t waypoint_index = wmgm_it->first;
        std::map<string,int> manip_name_group_index_map = wmgm_it->second;

        for(std::map<string,int>::iterator mngim_it = manip_name_group_index_map.begin(); mngim_it != manip_name_group_index_map.end(); mngim_it++)
        {
            string manip_name = mngim_it->first;
            int manip_group_index = mngim_it->second;

            if(_contact_manips_group.count(manip_group_index) != 0)
            {
                _contact_manips_group.find(manip_group_index)->second.waypoints.push_back(waypoint_index);
            }
            else
            {
                std::vector<size_t> waypoint_indices(1,waypoint_index);
                ContactManipGroup cmg(manip_group_index, manip_name, Transform(), waypoint_indices);
                _contact_manips_group.insert(std::pair<int,ContactManipGroup>(manip_group_index,cmg));
            }

            for(std::map<string,int>::iterator mngim_it_2 = manip_name_group_index_map.begin(); mngim_it_2 != manip_name_group_index_map.end(); mngim_it_2++)
            {
                if(manip_group_index != mngim_it_2->second)
                {
                    _contact_manips_group.find(manip_group_index)->second.avoid_contact_manip_group.insert(mngim_it_2->second);
                }
            }

        }
    }

    _start_contact_group_index.resize(2);
    _goal_contact_group_index.resize(2);

    bool first_left_foot_contact = true;
    bool first_right_foot_contact = true;

    for(std::map< int, ContactManipGroup >::iterator cmg_it = _contact_manips_group.begin(); cmg_it != _contact_manips_group.end(); cmg_it++)
    {
        int contact_manip_group_index = cmg_it->first;
        string manip_name = cmg_it->second.manip_name;

        if(strcmp(manip_name.c_str(),"l_leg") == 0)
        {
            if(first_left_foot_contact)
            {
                _start_contact_group_index[0] = contact_manip_group_index;
                first_left_foot_contact = false;
            }
            _goal_contact_group_index[0] = contact_manip_group_index;
        }
        else if(strcmp(manip_name.c_str(),"r_leg") == 0)
        {
            if(first_right_foot_contact)
            {
                _start_contact_group_index[1] = contact_manip_group_index;
                first_right_foot_contact = false;
            }
            _goal_contact_group_index[1] = contact_manip_group_index;
        }
    }

    int num_waypoints = _waypoint_manip_group_map.size();

    // std::cout << "num_waypoints=" << num_waypoints << std::endl;

    _cogtargs.resize(num_waypoints);
    _curcogs.resize(num_waypoints);
    _Jp.resize(num_waypoints);
    _Jp0.resize(num_waypoints);
    _Jr.resize(num_waypoints);
    _Jr0.resize(num_waypoints);
    _Jr_proper.resize(num_waypoints);
    _Jr_quat.resize(num_waypoints);
    _tooltm.resize(num_waypoints);
    _TMtool.resize(num_waypoints);
    _tasktm.resize(num_waypoints);
    _E_rpy.resize(num_waypoints);
    _E_rpy_inv.resize(num_waypoints);
    _TMtask.resize(num_waypoints);
    _JOA.resize(num_waypoints);
    _JOAplus.resize(num_waypoints);
    _doa.resize(num_waypoints);
    _JZ.resize(num_waypoints);
    _JRPY.resize(num_waypoints);
    _JZRPY.resize(num_waypoints);
    _JZRPYplus.resize(num_waypoints);
    _dzrpy.resize(num_waypoints);
    _JXY.resize(num_waypoints);
    _JXYplus.resize(num_waypoints);
    _dxy.resize(num_waypoints);
    _JXYZRPY.resize(num_waypoints);
    _JXYZRPYplus.resize(num_waypoints);
    _dxyzrpy.resize(num_waypoints);
    _JCOG.resize(num_waypoints);
    _JCOGplus.resize(num_waypoints);
    _dcog.resize(num_waypoints);
    _JPC.resize(num_waypoints);
    _JPCplus.resize(num_waypoints);
    _dpc.resize(num_waypoints);
    _JINT.resize(num_waypoints);
    _JINTplus.resize(num_waypoints);
    _dint.resize(num_waypoints);
    _Moa.resize(num_waypoints);
    _Moainv.resize(num_waypoints);
    _Mzrpy.resize(num_waypoints);
    _Mzrpyinv.resize(num_waypoints);
    _Mxy.resize(num_waypoints);
    _Mxyinv.resize(num_waypoints);
    _Mxyzrpy.resize(num_waypoints);
    _Mxyzrpyinv.resize(num_waypoints);
    _Mcog.resize(num_waypoints);
    _Mcoginv.resize(num_waypoints);
    _Mpc.resize(num_waypoints);
    _Mpcinv.resize(num_waypoints);
    _Mint.resize(num_waypoints);
    _Mintinv.resize(num_waypoints);
    _W.resize(num_waypoints);
    _Winv.resize(num_waypoints);
    _Regoa.resize(num_waypoints);
    _Regxy.resize(num_waypoints);
    _Regzrpy.resize(num_waypoints);
    _Regxyzrpy.resize(num_waypoints);
    _Regcog.resize(num_waypoints);
    _Regpc.resize(num_waypoints);
    _Regint.resize(num_waypoints);
    _JHP.resize(num_waypoints);
    _JHPplus.resize(num_waypoints);
    _dhp.resize(num_waypoints);
    _balance_step.resize(num_waypoints);
    _oa_step.resize(num_waypoints);
    _zrpy_step.resize(num_waypoints);
    _xy_step.resize(num_waypoints);
    _xyzrpy_step.resize(num_waypoints);
    _pc_step.resize(num_waypoints);
    _int_step.resize(num_waypoints);
    _step.resize(num_waypoints);

    _exclude_control_points.resize(num_waypoints);
    _links_in_collision.resize(num_waypoints);

    _xy_error.resize(num_waypoints);
    _z_error.resize(num_waypoints);
    _rpy_error.resize(num_waypoints);

    for(int w = 0; w < num_waypoints; w++)
    {
        auto manip_group_map = _waypoint_manip_group_map[w];
        int manip_num_in_group = manip_group_map.size();

        // XY
        _JXY[w].ReSize(2*manip_num_in_group,_numdofs);
        _JXYplus[w].ReSize(_numdofs,2*manip_num_in_group);
        _Mxy[w].ReSize(2*manip_num_in_group);
        _Mxyinv[w].ReSize(2*manip_num_in_group);
        _dxy[w].ReSize(2*manip_num_in_group);
        _Regxy[w].ReSize(2*manip_num_in_group);
        _Regxy[w] = 0.001;

        // Z
        _JZ[w].ReSize(1,_numdofs);
        _JRPY[w].ReSize(3,_numdofs);

        _JZRPY[w].ReSize(4*manip_num_in_group,_numdofs);
        _JZRPYplus[w].ReSize(_numdofs,4*manip_num_in_group);
        _Mzrpy[w].ReSize(4*manip_num_in_group);
        _Mzrpyinv[w].ReSize(4*manip_num_in_group);
        _dzrpy[w].ReSize(4*manip_num_in_group);
        _Regzrpy[w].ReSize(4*manip_num_in_group);
        _Regzrpy[w] = 0.001;

        _JXYZRPY[w].ReSize(6*manip_num_in_group,_numdofs);
        _JXYZRPYplus[w].ReSize(_numdofs,6*manip_num_in_group);
        _Mxyzrpy[w].ReSize(6*manip_num_in_group);
        _Mxyzrpyinv[w].ReSize(6*manip_num_in_group);
        _dxyzrpy[w].ReSize(6*manip_num_in_group);
        _Regxyzrpy[w].ReSize(6*manip_num_in_group);
        _Regxyzrpy[w] = 0.001;

        _balance_step[w].ReSize(_numdofs);
        _balance_step[w] = 0;
        _oa_step[w].ReSize(_numdofs);
        _oa_step[w] = 0;
        _zrpy_step[w].ReSize(_numdofs);
        _zrpy_step[w] = 0;
        _xy_step[w].ReSize(_numdofs);
        _xy_step[w] = 0;
        _pc_step[w].ReSize(_numdofs);
        _pc_step[w] = 0;
        // _m_step[w].ReSize(_numdofs);
        // _m_step[w] = 0;

        _tooltm[w].ReSize(3,3);
        _tasktm[w].ReSize(3,3);
        _E_rpy[w].ReSize(3,3);

        _Jp[w].ReSize(3,_numdofs);
        _Jp0[w].ReSize(3,_numdofs);

        _Jr[w].ReSize(3,_numdofs);
        _Jr0[w].ReSize(3,_numdofs);
        _Jr_proper[w].ReSize(3,_numdofs);

        _step[w].ReSize(_numdofs);
        _step[w] = 0.0;

        _W[w].ReSize(_numdofs);
        _Winv[w].ReSize(_numdofs);
        _W[w] = 1.0;
        _Winv[w] = 1.0;

        for(map<string,int>::iterator mgm_it = manip_group_map.begin(); mgm_it != manip_group_map.end(); mgm_it++)
        {
            string manip_name = mgm_it->first;
            int manip_group_index = mgm_it->second;

            if(strcmp(manip_name.c_str(),"l_arm") == 0)
            {
                _exclude_control_points[w].insert("l_palm");
                _exclude_control_points[w].insert("l_forearm");
            }
            else if(strcmp(manip_name.c_str(),"r_arm") == 0)
            {
                _exclude_control_points[w].insert("r_palm");
                _exclude_control_points[w].insert("r_forearm");
            }
            else if(strcmp(manip_name.c_str(),"l_leg") == 0)
            {
                _exclude_control_points[w].insert("l_foot");
                _exclude_control_points[w].insert("l_shin");
            }
            else if(strcmp(manip_name.c_str(),"r_leg") == 0)
            {
                _exclude_control_points[w].insert("r_foot");
                _exclude_control_points[w].insert("r_shin");
            }
        }
    }

}

void ElasticStrips::GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point, size_t w)
{
    RobotBasePtr robot = _esRobotVec[w%_num_robots];
    dReal repulse_constant = -1;
    Transform link_transform = robot->GetLink(control_point->first)->GetTransform();
    Vector control_point_global_position = link_transform*control_point->second;
    repulsive_vector = Vector(0,0,0);

    //environment collision
    for(std::vector< std::pair<KinBodyPtr,Vector> >::iterator obs_it = _parameters->_esObstacle.begin(); obs_it != _parameters->_esObstacle.end(); obs_it++)
    {
        KinBodyPtr ObstacleKinBody = obs_it->first;
        Vector obs_repulsive_vector = obs_it->second;

        if(GetEnv()->CheckCollision(robot->GetLink(control_point->first),ObstacleKinBody->GetLinks()[0]))
        {
            // std::vector<KinBody::Link::GeometryPtr> ObstacleGeometry = (*link_it)->GetGeometries();

            _links_in_collision[w].insert(control_point->first);

            repulsive_vector = repulsive_vector + obs_repulsive_vector;
        }
    }

    //self collision
    // for(std::vector<std::pair<string,string> >::iterator sc_it = _parameters->_self_collision_checking_pairs.begin(); sc_it != _parameters->_self_collision_checking_pairs.end(); sc_it++)
    // {
    //     string link_1 = (*sc_it).first;
    //     string link_2 = (*sc_it).second;
    //     if(control_point->first == link_1 || control_point->first == link_2)
    //     {
    //         if(GetEnv()->CheckCollision(robot->GetLink(link_1),robot->GetLink(link_2)))
    //         {
    //             RaveVector<dReal> repulsive_vector_component(0,0,0);
    //             string other_link = (control_point->first == link_1) ? link_2 : link_1;
    //             RaveVector<dReal> other_link_centroid = robot->GetLink(other_link)->GetTransform().trans;
    //             repulsive_vector_component = (control_point_global_position - other_link_centroid).normalize3();
    //             repulsive_vector = repulsive_vector + repulsive_vector_component;
    //             shortest_dist = 0;
    //         }
    //     }
    // }

    if(repulsive_vector.lengthsqr3() != 0)
    {
        repulsive_vector = repulse_constant * repulsive_vector * (1/sqrt(repulsive_vector.lengthsqr3()));
        // cout<<"Link: "<<control_point->first<<", Repulsive Vector: ("<<repulsive_vector.x<<","<<repulsive_vector.y<<","<<repulsive_vector.z<<")"<<endl;
    }

    //for each obstacle, find its geometry type
    //calculate distance between the control point and the obstacle
    //calculate the repulsive from each obstacle according to ther relative position and distance
    //sum the repulsive vector
}

// void ElasticStrips::GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point)
// {
//     dReal repulse_dist = 1000;
//     dReal repulse_constant = -1;
//     Transform link_transform = _esRobot->GetLink(control_point->first)->GetTransform();
//     Vector control_point_global_position = link_transform*control_point->second;
//     repulsive_vector = Vector(0,0,0);
//     dReal shortest_dist = 100000000;

//     //environment collision
//     for(std::vector<KinBodyPtr>::iterator obs_it = _parameters->_esObstacle.begin(); obs_it != _parameters->_esObstacle.end(); obs_it++)
//     {
//         std::vector<KinBody::LinkPtr> ObstacleLink = (*obs_it)->GetLinks();
//         for(std::vector<KinBody::LinkPtr>::iterator link_it = ObstacleLink.begin(); link_it != ObstacleLink.end(); link_it++)
//         {
//             if(GetEnv()->CheckCollision(_esRobot->GetLink(control_point->first),(*link_it)))
//             {
//                 std::vector<KinBody::Link::GeometryPtr> ObstacleGeometry = (*link_it)->GetGeometries();
//                 for(std::vector<KinBody::Link::GeometryPtr>::iterator geom_it = ObstacleGeometry.begin(); geom_it != ObstacleGeometry.end(); geom_it++)
//                 {
//                     GeometryType obstacle_geometry_type = (*geom_it)->GetType();
//                     RaveTransform<dReal> obstacle_geometry_transform = (*obs_it)->GetTransform() * (*link_it)->GetTransform() * (*geom_it)->GetTransform();
//                     RaveTransformMatrix<dReal> obstacle_rot_matrix = geometry::matrixFromQuat(obstacle_geometry_transform.rot);
//                     RaveTransformMatrix<dReal> inverse_obstacle_rot_matrix = obstacle_rot_matrix.inverse();
//                     RaveVector<dReal> obstacle_translation = obstacle_geometry_transform.trans;
//                     RaveVector<dReal> obstacle_frame_control_point_position = inverse_obstacle_rot_matrix * (control_point_global_position-obstacle_translation);
//                     dReal dist_to_obstacle = 0;
//                     RaveVector<dReal> repulsive_vector_component(0,0,0);
//                     RaveVector<dReal> nearest_point(0,0,0);
//                     if(obstacle_geometry_type == GT_Box)
//                     {
//                         Vector box_extents = (*geom_it)->GetBoxExtents();
//                         if(obstacle_frame_control_point_position.x > box_extents.x/2)
//                             nearest_point.x = box_extents.x/2;
//                         else if(obstacle_frame_control_point_position.x < -box_extents.x/2)
//                             nearest_point.x = -box_extents.x/2;
//                         else
//                             nearest_point.x = obstacle_frame_control_point_position.x;

//                         if(obstacle_frame_control_point_position.y > box_extents.y/2)
//                             nearest_point.y = box_extents.y/2;
//                         else if(obstacle_frame_control_point_position.y < -box_extents.y/2)
//                             nearest_point.y = -box_extents.y/2;
//                         else
//                             nearest_point.y = obstacle_frame_control_point_position.y;

//                         if(obstacle_frame_control_point_position.z > box_extents.z/2)
//                             nearest_point.z = box_extents.z/2;
//                         else if(obstacle_frame_control_point_position.z < -box_extents.z/2)
//                             nearest_point.z = -box_extents.z/2;
//                         else
//                             nearest_point.z = obstacle_frame_control_point_position.z;

//                         if(obstacle_frame_control_point_position.x != nearest_point.x ||
//                            obstacle_frame_control_point_position.y != nearest_point.y ||
//                            obstacle_frame_control_point_position.z != nearest_point.z)
//                         {
//                             repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
//                             dist_to_obstacle = sqrt(repulsive_vector_component.lengthsqr3());
//                             repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
//                         }
//                         else
//                         {
//                             nearest_point = RaveVector<dReal>(0,0,0);
//                             repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
//                             repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
//                             dist_to_obstacle = 0;
//                         }

//                     }
//                     else if(obstacle_geometry_type == GT_Sphere)
//                     {
//                         repulsive_vector_component = control_point_global_position - obstacle_translation;
//                         if(sqrt(repulsive_vector_component.lengthsqr3()) > (*geom_it)->GetSphereRadius())
//                             dist_to_obstacle = sqrt(repulsive_vector_component.lengthsqr3()) - (*geom_it)->GetSphereRadius();
//                         else
//                             dist_to_obstacle = 0;
//                         repulsive_vector_component = repulsive_vector_component.normalize3();
//                     }
//                     else if(obstacle_geometry_type == GT_Cylinder)
//                     {
//                         dReal cylinder_height = (*geom_it)->GetCylinderHeight();
//                         dReal cylinder_radius = (*geom_it)->GetCylinderRadius();
//                         //RaveVector<dReal> nearest_point(0,0,0);
//                         dReal xy_dist_to_centroid = sqrt(pow(obstacle_frame_control_point_position.x,2) + pow(obstacle_frame_control_point_position.y,2));

//                         if(xy_dist_to_centroid > cylinder_radius || fabs(obstacle_frame_control_point_position.z) > cylinder_height/2)
//                         {
//                             nearest_point.x = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.x;
//                             nearest_point.y = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.y;

//                             if(obstacle_frame_control_point_position.z > cylinder_height/2)
//                                 nearest_point.z = cylinder_height/2;
//                             else if(obstacle_frame_control_point_position.z < -cylinder_height/2)
//                                 nearest_point.z = -cylinder_height/2;
//                             else
//                                 nearest_point.z = obstacle_frame_control_point_position.z;

//                             repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
//                             dist_to_obstacle = sqrt(repulsive_vector_component.lengthsqr3());
//                         }
//                         else
//                         {
//                             nearest_point = RaveVector<dReal>(0,0,0);
//                             repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
//                             dist_to_obstacle = 0;
//                         }

//                         repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
//                     }

//                     if(dist_to_obstacle < repulse_dist)
//                         repulsive_vector = repulsive_vector + repulsive_vector_component;

//                     if(dist_to_obstacle < shortest_dist)
//                         shortest_dist = dist_to_obstacle;

//                 }

//             }
//         }
//     }

//     //self collision
//     for(std::vector<std::pair<string,string> >::iterator sc_it = _parameters->_self_collision_checking_pairs.begin(); sc_it != _parameters->_self_collision_checking_pairs.end(); sc_it++)
//     {
//         string link_1 = (*sc_it).first;
//         string link_2 = (*sc_it).second;
//         if(control_point->first == link_1 || control_point->first == link_2)
//         {
//             if(GetEnv()->CheckCollision(_esRobot->GetLink(link_1),_esRobot->GetLink(link_2)))
//             {
//                 RaveVector<dReal> repulsive_vector_component(0,0,0);
//                 string other_link = (control_point->first == link_1) ? link_2 : link_1;
//                 RaveVector<dReal> other_link_centroid = _esRobot->GetLink(other_link)->GetTransform().trans;
//                 repulsive_vector_component = (control_point_global_position - other_link_centroid).normalize3();
//                 repulsive_vector = repulsive_vector + repulsive_vector_component;
//                 shortest_dist = 0;
//             }
//         }
//     }

//     if(repulsive_vector.lengthsqr3() != 0)
//     {
//         repulsive_vector = repulse_constant * exp(-shortest_dist) * repulsive_vector * (1/sqrt(repulsive_vector.lengthsqr3()));
//         // cout<<"Link: "<<control_point->first<<", Repulsive Vector: ("<<repulsive_vector.x<<","<<repulsive_vector.y<<","<<repulsive_vector.z<<")"<<endl;
//     }

//     //for each obstacle, find its geometry type
//     //calculate distance between the control point and the obstacle
//     //calculate the repulsive from each obstacle according to ther relative position and distance
//     //sum the repulsive vector
// }

void ElasticStrips::RemoveBadJointJacobianCols(NEWMAT::Matrix& J, size_t w)
{
    if(!_badjointinds.at(w).empty())
    {
        for(int j = 0; j < _badjointinds.at(w).size(); j++)
            for(int k = 1; k <= J.Nrows(); k++)
                J(k, _badjointinds.at(w).at(j)+1) = 0;
    }
}

void ElasticStrips::UpdateZRPYandXYJacobianandStep(Transform taskframe_in, size_t w)
{
    RobotBasePtr robot = _esRobotVec[w%_num_robots];
    map<string,int> manip_group_map = _waypoint_manip_group_map.find(w)->second;
    map<string,int> prev_manip_group_map = manip_group_map;
    string new_contact_manip = "";

    if(w != 0)
    {
        prev_manip_group_map = _waypoint_manip_group_map.find(w-1)->second;
    }

    for(map<string,int>::iterator it = manip_group_map.begin(); it != manip_group_map.end(); it++)
    {
        if(prev_manip_group_map.count(it->first) == 0)
        {
            new_contact_manip = it->first;
            break;
            // only 1 new contact manip
        }
    }

    _cogtargs[w].x = 0; _cogtargs[w].y = 0; _cogtargs[w].z = 0;

    Transform manip_offset_transform;

    int i = 0;
    vector<string> support_links;
    vector<string> support_manips;
    vector<dReal> support_mus;

    std::vector<GraphHandlePtr> handles;

    dReal psi, theta, phi;
    dReal Cphi,Ctheta,Cpsi,Sphi,Stheta,Spsi;

    for(map<string,int>::iterator mgm_it = manip_group_map.begin(); mgm_it != manip_group_map.end(); mgm_it++, i++)
    {
        string manip_name = mgm_it->first;
        int manip_group_index = mgm_it->second;

        if(strcmp(manip_name.c_str(),"l_arm") == 0)
        {
            manip_offset_transform = _l_arm_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"r_arm") == 0)
        {
            manip_offset_transform = _r_arm_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"l_leg") == 0)
        {
            manip_offset_transform = _l_leg_manip_transform_offset;
        }
        else if(strcmp(manip_name.c_str(),"r_leg") == 0)
        {
            manip_offset_transform = _r_leg_manip_transform_offset;
        }

        // int region_index = contact_consistent_region.find(w)->second.find(manip_name)->second;
        // Vector contact_consistent_point = contact_consistent_manip_translation.at(region_index).second;

        // Transform contact_consistent_point_frame = ncr_it->second.contact_region_frame;

        // contact_consistent_point_frame.trans.x = contact_consistent_point.x;
        // contact_consistent_point_frame.trans.y = contact_consistent_point.y;
        // contact_consistent_point_frame.trans.z = contact_consistent_point.z;

        Transform contact_consistent_point_frame = _contact_manips_group.find(manip_group_index)->second.contact_consistent_transform; // contact_consistent_point_frame with rotated end-effector transform

        // Jacobian
        RobotBase::ManipulatorPtr target_manip = robot->GetManipulators()[_GetManipIndex.find(manip_name)->second];
        std::vector<dReal> temp;

        robot->CalculateActiveJacobian(target_manip->GetEndEffector()->GetIndex(), target_manip->GetTransform().trans, temp);
        memcpy(_Jp0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

        _TMtool[w] = manip_offset_transform;
        _tooltm[w](1,1) = _TMtool[w].m[0];        _tooltm[w](1,2) = _TMtool[w].m[1];        _tooltm[w](1,3) = _TMtool[w].m[2];
        _tooltm[w](2,1) = _TMtool[w].m[4];        _tooltm[w](2,2) = _TMtool[w].m[5];        _tooltm[w](2,3) = _TMtool[w].m[6];
        _tooltm[w](3,1) = _TMtool[w].m[8];        _tooltm[w](3,2) = _TMtool[w].m[9];        _tooltm[w](3,3) = _TMtool[w].m[10];

        robot->CalculateActiveAngularVelocityJacobian(target_manip->GetEndEffector()->GetIndex(), temp);
        memcpy(_Jr0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

        _TMtask[w] = contact_consistent_point_frame.inverse() * taskframe_in.inverse();
        _tasktm[w](1,1) = _TMtask[w].m[0];        _tasktm[w](1,2) = _TMtask[w].m[1];        _tasktm[w](1,3) = _TMtask[w].m[2];
        _tasktm[w](2,1) = _TMtask[w].m[4];        _tasktm[w](2,2) = _TMtask[w].m[5];        _tasktm[w](2,3) = _TMtask[w].m[6];
        _tasktm[w](3,1) = _TMtask[w].m[8];        _tasktm[w](3,2) = _TMtask[w].m[9];        _tasktm[w](3,3) = _TMtask[w].m[10];

        _Jp[w] = _tasktm[w] * _Jp0[w];
        _Jr[w] = _tasktm[w] * (-_Jr0[w]);

        // _JXY[w].Rows(i*2+1,i*2+2) = _Jp[w].Rows(1,2);
        // _JZ[w].Row(1) = _Jp[w].Row(3);

        //convert current rotation to euler angles (RPY)
        QuatToRPY(contact_consistent_point_frame.inverse()*taskframe_in.inverse()*(target_manip->GetTransform()*manip_offset_transform),psi,theta,phi);
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(phi);
        Ctheta = cos(theta);
        Cpsi = cos(psi);

        Sphi = sin(phi);
        Stheta = sin(theta);
        Spsi = sin(psi);

        _E_rpy[w](1,1) = Cphi/Ctheta;         _E_rpy[w](1,2) = Sphi/Ctheta;         _E_rpy[w](1,3) = 0;
        _E_rpy[w](2,1) = -Sphi;               _E_rpy[w](2,2) = Cphi;                _E_rpy[w](2,3) = 0;
        _E_rpy[w](3,1) = Cphi*Stheta/Ctheta;  _E_rpy[w](3,2) = Sphi*Stheta/Ctheta;  _E_rpy[w](3,3) = 1;

        _Jr_proper[w] = _E_rpy[w] * _Jr[w];

        _JRPY[w] = -_Jr_proper[w];

        // _JZRPY[w].Row(i*4+1) = _JZ[w].Row(1);
        // _JZRPY[w].Rows(i*4+2,i*4+4) = _JRPY[w].Rows(1,3);

        _JXYZRPY[w].Rows(i*6+1,i*6+3) = _Jp[w].Rows(1,3);
        _JXYZRPY[w].Rows(i*6+4,i*6+6) = _JRPY[w].Rows(1,3);

        // Step
        NEWMAT::ColumnVector ds;
        ds.ReSize(6);

        TransformDifference(ds.Store(), contact_consistent_point_frame, target_manip->GetTransform()*manip_offset_transform);

        //velocity, can be written as force
        // _dxy[w](i*2+1) = ds(1);
        // _dxy[w](i*2+2) = ds(2);
        // _dzrpy[w](i*4+1) = ds(3);
        // _dzrpy[w](i*4+2) = ds(4);
        // _dzrpy[w](i*4+3) = ds(5);
        // _dzrpy[w](i*4+4) = ds(6);

        _dxyzrpy[w](i*6+1) = ds(1);
        _dxyzrpy[w](i*6+2) = ds(2);
        _dxyzrpy[w](i*6+3) = ds(3);
        _dxyzrpy[w](i*6+4) = ds(4);
        _dxyzrpy[w](i*6+5) = ds(5);
        _dxyzrpy[w](i*6+6) = ds(6);

        _xy_error[w] = _xy_error[w] + (ds(1) * ds(1)) + (ds(2) * ds(2));
        _z_error[w] = _z_error[w] + (ds(3) * ds(3));
        _rpy_error[w] = _rpy_error[w] + (ds(4) * ds(4)) + (ds(5) * ds(5)) + (ds(6) * ds(6));;

        //dirty code to decide cogtarg:(specialize to escher robot)
        // cout << "manip_name: "<<manip_name<<", new_contact_manip: "<<new_contact_manip<<endl;
        // for(vector<string>::iterator sl_it = support_links.begin(); sl_it != support_links.end(); sl_it++)
        // {
        //     cout << (*sl_it) << ", ";
        // }
        // cout << endl;

        if(strcmp(manip_name.c_str(),new_contact_manip.c_str()) != 0)
        {
            if(strcmp(manip_name.c_str(),"l_leg") == 0)
            {
                _cogtargs[w].x += contact_consistent_point_frame.trans.x;
                _cogtargs[w].y += contact_consistent_point_frame.trans.y;
                _cogtargs[w].z += contact_consistent_point_frame.trans.z;
                support_links.push_back("l_foot");
            }
            else if(strcmp(manip_name.c_str(),"r_leg") == 0)
            {
                _cogtargs[w].x += contact_consistent_point_frame.trans.x;
                _cogtargs[w].y += contact_consistent_point_frame.trans.y;
                _cogtargs[w].z += contact_consistent_point_frame.trans.z;
                support_links.push_back("r_foot");
            }

            if(_parameters->balance_mode == BALANCE_GIWC)
            {
                support_manips.push_back(manip_name);
            }
        }

        //************//
        // Transform contact_consistent_transform = contact_consistent_point_frame;

        // Vector cct_origin = contact_consistent_transform.trans;
        // TransformMatrix contact_consistent_transform_matrix = TransformMatrix(contact_consistent_transform);
        // Vector cct_x = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[0],contact_consistent_transform_matrix.m[4],contact_consistent_transform_matrix.m[8]);
        // Vector cct_y = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[1],contact_consistent_transform_matrix.m[5],contact_consistent_transform_matrix.m[9]);
        // Vector cct_z = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[2],contact_consistent_transform_matrix.m[6],contact_consistent_transform_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

        // Vector center_to_center_translation(ds(1),ds(2),ds(3));
        // center_to_center_translation = contact_consistent_transform.rotate(center_to_center_translation);

        // handles.push_back(_esEnv->drawarrow(cct_origin, cct_origin+center_to_center_translation, 0.008, RaveVector<float>(1, 1, 0, 1)));

        // Transform ee_transform = target_manip->GetTransform();
        // Vector ee_origin = ee_transform.trans;
        // TransformMatrix ee_transform_matrix = TransformMatrix(ee_transform);
        // Vector ee_x = ee_transform.trans + 0.5 * Vector(ee_transform_matrix.m[0],ee_transform_matrix.m[4],ee_transform_matrix.m[8]);
        // Vector ee_y = ee_transform.trans + 0.5 * Vector(ee_transform_matrix.m[1],ee_transform_matrix.m[5],ee_transform_matrix.m[9]);
        // Vector ee_z = ee_transform.trans + 0.5 * Vector(ee_transform_matrix.m[2],ee_transform_matrix.m[6],ee_transform_matrix.m[10]);

        // handles.push_back(_esEnv->drawarrow(ee_origin, ee_x, 0.008, RaveVector<float>(1, 0.5, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(ee_origin, ee_y, 0.008, RaveVector<float>(0.5, 1, 0, 1)));
        // handles.push_back(_esEnv->drawarrow(ee_origin, ee_z, 0.008, RaveVector<float>(0.5, 0, 1, 1)));

        // getchar();
    }

    // RemoveBadJointJacobianCols(_JXY[w],w);
    // _Mxy[w] << _JXY[w]*_JXY[w].t() + _Regxy[w];
    // invConditioningBound(10000,_Mxy[w],_Mxyinv[w]);
    // _JXYplus[w] = _JXY[w].t()*_Mxyinv[w];

    // RemoveBadJointJacobianCols(_JZRPY[w],w);
    // _Mzrpy[w] << _JZRPY[w]*_JZRPY[w].t() + _Regzrpy[w];
    // invConditioningBound(10000,_Mzrpy[w],_Mzrpyinv[w]);
    // _JZRPYplus[w] = _JZRPY[w].t()*_Mzrpyinv[w];

    RemoveBadJointJacobianCols(_JXYZRPY[w],w);
    _Mxyzrpy[w] << _JXYZRPY[w]*_JXYZRPY[w].t() + _Regxyzrpy[w];
    invConditioningBound(10000,_Mxyzrpy[w],_Mxyzrpyinv[w]);
    _JXYZRPYplus[w] = _JXYZRPY[w].t()*_Mxyzrpyinv[w];

    _cogtargs[w].x /= support_links.size();
    _cogtargs[w].y /= support_links.size();
    _cogtargs[w].z /= support_links.size();
    _cogtargs[w].z = _cogtargs[w].z + 0.7;
    if(support_links.size() == 0)
    {
        cout<<"support_links.size == 0, bug!!!!"<<endl;
        getchar();
    }
    // if(support_links.size() == 1) // standing with one foot
    // {
    //     Vector l_foot_transform = robot->GetLink("l_foot")->GetTransform().trans;
    //     Vector r_foot_transform = robot->GetLink("r_foot")->GetTransform().trans;
    //     cogtarg.x = (l_foot_transform.x + r_foot_transform.x)/2.0;
    // }

    // if(_parameters->balance_mode == BALANCE_GIWC)
    // {
    //     cout<<"desired manips: ";
    //     for(std::map<string,int>::iterator mgm_it = manip_group_map.begin(); mgm_it != manip_group_map.end(); mgm_it++)
    //     {
    //         cout<<mgm_it->first<<" ";
    //     }
    //     cout<<endl;
    //     cout<<"contact manips: ";
    //     for(std::vector<string>::iterator sm_it = support_manips.begin(); sm_it != support_manips.end(); sm_it++)
    //     {
    //         cout<<*sm_it<<" ";
    //     }
    //     cout<<endl;
    //     getchar();
    // }

    vector<dReal> qs;
    robot->GetActiveDOFValues(qs);
    if(_parameters->balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        _balance_checkers[w]->RefreshBalanceParameters(qs,support_links,_giwc_database);
    }
    else if(_parameters->balance_mode == BALANCE_GIWC)
    {
        _balance_checkers[w]->RefreshBalanceParameters(qs,support_manips,_giwc_database);
    }


    _xy_error[w] = sqrt(_xy_error[w]);
    _z_error[w] = sqrt(_z_error[w]);
    _rpy_error[w] = sqrt(_rpy_error[w]);

    // dxy = cxy * dxy;
    // dzrpy = czrpy * dzrpy;

}


void ElasticStrips::UpdateCOGJacobianandStep(Transform taskframe_in,size_t w)
{
    RobotBasePtr robot = _esRobotVec[w%_num_robots];

    _JCOG[w] = 0.0;

    _curcogs[w].x = 0; _curcogs[w].y = 0; _curcogs[w].z = 0;
    dReal fTotalMass = 0;
    std::vector<KinBody::LinkPtr>::const_iterator itlink;

    TransformMatrix TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm[w](1,1) = TMtask.m[0];        _tasktm[w](1,2) = TMtask.m[1];        _tasktm[w](1,3) = TMtask.m[2];
    _tasktm[w](2,1) = TMtask.m[4];        _tasktm[w](2,2) = TMtask.m[5];        _tasktm[w](2,3) = TMtask.m[6];
    _tasktm[w](3,1) = TMtask.m[8];        _tasktm[w](3,2) = TMtask.m[9];        _tasktm[w](3,3) = TMtask.m[10];


    std::vector<dReal> temp;
    FORIT(itlink, robot->GetLinks())
    {
        robot->CalculateActiveJacobian((*itlink)->GetIndex(), ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset()), temp);
        memcpy(_Jp0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

        _Jp[w] = _tasktm[w] * _Jp0[w];

        _JCOG[w] = _JCOG[w] + ((*itlink)->GetMass()*(_Jp[w].Rows(1,2)));
        // JCOG = JCOG + ((*itlink)->GetMass()*(_Jp.Rows(1,3)));

        _curcogs[w] += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        _curcogs[w] /= fTotalMass;

    _JCOG[w] = _JCOG[w]/fTotalMass;
    RemoveBadJointJacobianCols(_JCOG[w],w);
    _Mcog[w] << _JCOG[w]*_JCOG[w].t() + _Regcog[w];
    invConditioningBound(10000,_Mcog[w],_Mcoginv[w]);
    _JCOGplus[w] = _JCOG[w].t()*_Mcoginv[w];

    if(!_balance_checkers[w]->CheckSupport(_curcogs[w]))
    {
        // if(_parameters->balance_mode == BALANCE_SUPPORT_POLYGON)
        //     cogtarg.z = curcog.z;

        _dcog[w](1) = (_curcogs[w].x - _cogtargs[w].x);
        _dcog[w](2) = (_curcogs[w].y - _cogtargs[w].y);
        // _dcog[w](3) = (_curcogs[w].y - _cogtargs[w].y);
    }
    else
    {
        _dcog[w](1) = 0;
        _dcog[w](2) = 0;
        // _dcog[w](3) = 0;
    }

    _dcog[w] = _ccog * _dcog[w];

}

void ElasticStrips::UpdateOAJacobianandStep(Transform taskframe_in, TrajectoryBasePtr ptraj, size_t w, bool& bInCollision)
{
    //calculate the velocity of each control point
    //calculate jacobian for each control point to generate the joint angular velocity
    //sum the joint angular velocity
    RobotBasePtr robot = _esRobotVec[w%_num_robots];

    std::map<string,Vector> control_points_in_movement;
    std::map<string,Vector> control_point_movement_vector;
    std::map<string,NEWMAT::Matrix> control_point_jacobian;

    bInCollision = false;

    _links_in_collision[w].clear();

    for(std::map<string,Vector>::iterator ctrl_it = _parameters->_control_points.begin(); ctrl_it != _parameters->_control_points.end(); ctrl_it++)
    {
        // cout<<ctrl_it->first<<endl;
        if(_exclude_control_points[w].count(ctrl_it->first) == 1)
        {
            // cout<<"exclude: "<<ctrl_it->first<<endl;
            continue;
        }

        Vector repulsive_vector(0,0,0);
        GetRepulsiveVector(repulsive_vector, ctrl_it, w);

        if(repulsive_vector.lengthsqr3() != 0)
        {
            bInCollision = true;
        }

        if(repulsive_vector.lengthsqr3() != 0)
        {
            _Jp[w].ReSize(3,_numdofs);

            _TMtask[w] = TransformMatrix(taskframe_in.inverse());
            _tasktm[w](1,1) = _TMtask[w].m[0];        _tasktm[w](1,2) = _TMtask[w].m[1];        _tasktm[w](1,3) = _TMtask[w].m[2];
            _tasktm[w](2,1) = _TMtask[w].m[4];        _tasktm[w](2,2) = _TMtask[w].m[5];        _tasktm[w](2,3) = _TMtask[w].m[6];
            _tasktm[w](3,1) = _TMtask[w].m[8];        _tasktm[w](3,2) = _TMtask[w].m[9];        _tasktm[w](3,3) = _TMtask[w].m[10];

            KinBody::LinkPtr target_link = robot->GetLink(ctrl_it->first);
            std::vector<dReal> temp;

            robot->CalculateActiveJacobian(target_link->GetIndex(), (target_link->GetTransform() * ctrl_it->second), temp);
            memcpy(_Jp0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

            _Jp[w] = _tasktm[w] * _Jp0[w];

            control_points_in_movement.insert(*ctrl_it);
            control_point_movement_vector.insert(std::pair<string,Vector>(ctrl_it->first,repulsive_vector));
            control_point_jacobian.insert(std::pair<string,NEWMAT::Matrix>(ctrl_it->first,_Jp[w]));
        }
    }

    if(!control_points_in_movement.empty())
    {
        NEWMAT::Matrix Jtemp;
        Jtemp.ReSize(0,_numdofs);
        _doa[w].ReSize(3*control_points_in_movement.size());

        int point_index = 0;

        //stack the repulsive vector and jacobian matrix
        for(std::map<string,Vector>::iterator ctrl_it = control_points_in_movement.begin(); ctrl_it != control_points_in_movement.end(); ctrl_it++)
        {
            Jtemp &= control_point_jacobian.find(ctrl_it->first)->second;
            _doa[w](point_index*3+1) = control_point_movement_vector.find(ctrl_it->first)->second.x;
            _doa[w](point_index*3+2) = control_point_movement_vector.find(ctrl_it->first)->second.y;
            _doa[w](point_index*3+3) = control_point_movement_vector.find(ctrl_it->first)->second.z;
            point_index++;
        }

        _JOA[w] = Jtemp;

        RemoveBadJointJacobianCols(_JOA[w],w);
        _Regoa[w].ReSize(_JOA[w].Nrows());
        _Regoa[w] = 0.001;
        _Moa[w].ReSize(_JOA[w].Nrows());
        _Moainv[w].ReSize(_JOA[w].Nrows());
        _Moa[w] << (_JOA[w]*_JOA[w].t()) + _Regoa[w];
        invConditioningBound(10000,_Moa[w],_Moainv[w]);
        _JOAplus[w].ReSize(_numdofs,3*control_points_in_movement.size());
        _JOAplus[w] = _JOA[w].t()*_Moainv[w];

        // doa = coa * doa / control_points_in_movement.size();
        _doa[w] = _coa * _doa[w];
    }
    else
    {
        _doa[w].ReSize(1);
        _JOA[w].ReSize(1,_numdofs);
        _JOAplus[w].ReSize(_numdofs,1);
        _doa[w] = 0.0;
        _JOA[w] = 0.0;
        _JOAplus[w] = 0.0;
    }

}


void ElasticStrips::UpdatePCJacobianandStep(Transform taskframe_in,size_t w)
{
    RobotBasePtr robot = _esRobotVec[w%_num_robots];

    NEWMAT::Matrix Jtemp;
    Jtemp.ReSize(0,_numdofs);
    int i = 0;

    dReal psi, theta, phi;
    dReal Cphi,Ctheta,Cpsi,Sphi,Stheta,Spsi;

    for(map<string,Transform>::iterator pc_it = _parameters->_posture_control.begin(); pc_it != _parameters->_posture_control.end(); pc_it++, i++)
    {
        KinBody::LinkPtr target_link = robot->GetLink(pc_it->first);

        // Jacobian
        std::vector<dReal> temp;

        robot->CalculateActiveJacobian(target_link->GetIndex(), target_link->GetTransform().trans, temp);
        memcpy(_Jp0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

        robot->CalculateActiveAngularVelocityJacobian(target_link->GetIndex(), temp);
        memcpy(_Jr0[w].Store(),&temp[0],temp.size()*sizeof(dReal));


        _TMtask[w] = TransformMatrix(taskframe_in.inverse());
        _tasktm[w](1,1) = _TMtask[w].m[0];        _tasktm[w](1,2) = _TMtask[w].m[1];        _tasktm[w](1,3) = _TMtask[w].m[2];
        _tasktm[w](2,1) = _TMtask[w].m[4];        _tasktm[w](2,2) = _TMtask[w].m[5];        _tasktm[w](2,3) = _TMtask[w].m[6];
        _tasktm[w](3,1) = _TMtask[w].m[8];        _tasktm[w](3,2) = _TMtask[w].m[9];        _tasktm[w](3,3) = _TMtask[w].m[10];

        _Jp[w] = _tasktm[w] * _Jp0[w];

        _Jr[w] = _tasktm[w] * (-_Jr0[w]);

        //convert current rotation to euler angles (RPY)
        QuatToRPY(taskframe_in.inverse()*target_link->GetTransform(), psi, theta, phi);
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(phi);
        Ctheta = cos(theta);
        Cpsi = cos(psi);

        Sphi = sin(phi);
        Stheta = sin(theta);
        Spsi = sin(psi);

        _E_rpy[w](1,1) = Cphi/Ctheta;         _E_rpy[w](1,2) = Sphi/Ctheta;         _E_rpy[w](1,3) = 0;
        _E_rpy[w](2,1) = -Sphi;               _E_rpy[w](2,2) = Cphi;                _E_rpy[w](2,3) = 0;
        _E_rpy[w](3,1) = Cphi*Stheta/Ctheta;  _E_rpy[w](3,2) = Sphi*Stheta/Ctheta;  _E_rpy[w](3,3) = 1;

        _Jr_proper[w] = _E_rpy[w] * _Jr[w];

        Jtemp &= _Jp[w];
        Jtemp &= -_Jr_proper[w];


        // Step
        Transform desired_link_transform = taskframe_in.inverse() * pc_it->second * robot->GetLink("torso")->GetTransform();
        TransformDifference(_dpc[w].Store() + i*6, desired_link_transform, taskframe_in.inverse()*target_link->GetTransform());
    }

    _JPC[w] = Jtemp;
    RemoveBadJointJacobianCols(_JPC[w],w);
    _Mpc[w] << (_JPC[w]*_JPC[w].t()) + _Regpc[w];
    invConditioningBound(10000,_Mpc[w],_Mpcinv[w]);
    _JPCplus[w] = _JPC[w].t()*_Mpcinv[w];

    _dpc[w] = _cpc * _dpc[w];

}

void ElasticStrips::UpdateINTJacobianandStep(Transform taskframe_in, TrajectoryBasePtr ptraj, size_t w)
{
    RobotBasePtr robot = _esRobotVec[w%_num_robots];

    NEWMAT::Matrix Jtemp;
    Jtemp.ReSize(0,_numdofs);
    int i = 0;
    for(vector<string>::iterator int_it = _parameters->_internal_force_links.begin(); int_it != _parameters->_internal_force_links.end(); int_it++, i++)
    {
        KinBody::LinkPtr target_link = robot->GetLink(*int_it);

        // Jacobian
        std::vector<dReal> temp;

        robot->CalculateActiveJacobian(target_link->GetIndex(), target_link->GetTransform().trans, temp);
        memcpy(_Jp0[w].Store(),&temp[0],temp.size()*sizeof(dReal));

        _TMtask[w] = TransformMatrix(taskframe_in.inverse());
        _tasktm[w](1,1) = _TMtask[w].m[0];        _tasktm[w](1,2) = _TMtask[w].m[1];        _tasktm[w](1,3) = _TMtask[w].m[2];
        _tasktm[w](2,1) = _TMtask[w].m[4];        _tasktm[w](2,2) = _TMtask[w].m[5];        _tasktm[w](2,3) = _TMtask[w].m[6];
        _tasktm[w](3,1) = _TMtask[w].m[8];        _tasktm[w](3,2) = _TMtask[w].m[9];        _tasktm[w](3,3) = _TMtask[w].m[10];

        _Jp[w] = _tasktm[w] * _Jp0[w];

        Jtemp &= _Jp[w];

        // Step
        Vector internal_vector(0,0,0);
        GetInternalVector(internal_vector, ptraj, robot, *int_it, w);
        _dint[w](i*3+1) = internal_vector.x;
        _dint[w](i*3+2) = internal_vector.y;
        _dint[w](i*3+3) = internal_vector.z;
    }

    _JINT[w] = Jtemp;
    RemoveBadJointJacobianCols(_JINT[w],w);
    _Mint[w] << (_JINT[w]*_JINT[w].t()) + _Regint[w];
    invConditioningBound(10000,_Mint[w],_Mintinv[w]);
    _JINTplus[w] = _JINT[w].t()*_Mintinv[w];

    _dint[w] = _cint * _dint[w];

}

void ElasticStrips::GetInternalVector(Vector& internal_vector, TrajectoryBasePtr ptraj, RobotBasePtr robot, string control_link, size_t w)
{
    // Can be modified to include orientation.

    // EnvironmentMutex::scoped_lock lockenv(_esEnv->GetMutex());

    Vector prev_trans(0,0,0);
    Vector next_trans(0,0,0);
    Vector current_trans(0,0,0);
    vector<dReal> qs_prev(_numdofs);
    vector<dReal> qs(_numdofs);
    vector<dReal> qs_next(_numdofs);

    dReal dpc_trans = 0.0;
    dReal dcn_trans = 0.0;

    if(w != 0 && w != ptraj->GetNumWaypoints()-1)
    {
        ptraj->GetWaypoint(w,qs);
        robot->SetActiveDOFValues(qs);
        current_trans = robot->GetLink(control_link)->GetTransform().trans;

        ptraj->GetWaypoint(w-1,qs_prev);
        robot->SetActiveDOFValues(qs_prev);
        prev_trans = robot->GetLink(control_link)->GetTransform().trans;
        dpc_trans = sqrt((current_trans-prev_trans).lengthsqr3());

        ptraj->GetWaypoint(w+1,qs_next);
        robot->SetActiveDOFValues(qs_next);
        next_trans = robot->GetLink(control_link)->GetTransform().trans;
        dcn_trans = sqrt((next_trans-current_trans).lengthsqr3());

        if(dpc_trans+dcn_trans < 0.0001)
        {
            internal_vector = Vector(0,0,0);
        }
        else
        {
            internal_vector = -1.0 * ((dpc_trans/(dpc_trans+dcn_trans))*(next_trans-prev_trans) - (current_trans-prev_trans));
        }

        // reset the robot configuration
        robot->SetActiveDOFValues(qs);
    }
    else
    {
        internal_vector = Vector(0,0,0);
    }
}


dReal ElasticStrips::TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ)
{
    //used in transform difference
    Transform tmtemp;
    dReal sumsqr;

    tmtemp = tm_ref.inverse()*tm_targ;

    dx[0] = tmtemp.trans.x;
    dx[1] = tmtemp.trans.y;
    dx[2] = tmtemp.trans.z;

    QuatToRPY(tmtemp,dx[3],dx[4],dx[5]);

    sumsqr = 0;
    for(int i = 0; i < 6; i++)
        sumsqr += dx[i]*dx[i];

    return sqrt(sumsqr);
}

OpenRAVE::PlannerStatus ElasticStrips::PlanPath(TrajectoryBasePtr ptraj)
{
    EnvironmentMutex::scoped_lock lockenv(_esEnv->GetMutex());

    OpenRAVE::PlannerStatus result = PS_HasSolution;

    if(bPrint)
        RAVELOG_INFO("Initialize posture control and balance variables.\n");

    // Initialize invaraint Jacobian dimension of each constriant.
    for(unsigned int w = 0; w < ptraj->GetNumWaypoints(); w++)
    {
        if(_parameters->bPOSTURE_CONTROL)
        {
            _JPC[w].ReSize(6*_parameters->_posture_control.size(),_numdofs);
            _JPCplus[w].ReSize(_numdofs,6*_parameters->_posture_control.size());
            _Mpc[w].ReSize(6*_parameters->_posture_control.size());
            _Mpcinv[w].ReSize(6*_parameters->_posture_control.size());
            _Regpc[w].ReSize(6*_parameters->_posture_control.size());
            _Regpc[w] = 0.001;
            _dpc[w].ReSize(6*_parameters->_posture_control.size());
        }

        if(_parameters->balance_mode != BALANCE_NONE)
        {
            _JCOG[w].ReSize(2,_numdofs);
            _JCOGplus[w].ReSize(_numdofs,2);
            _Mcog[w].ReSize(2);
            _Mcoginv[w].ReSize(2);
            _Regcog[w].ReSize(2);
            _Regcog[w] = 0.001;
            _dcog[w].ReSize(2);
            // _JCOG[w].ReSize(3,_numdofs);
            // _JCOGplus[w].ReSize(_numdofs,3);
            // _Mcog[w].ReSize(3);
            // _Mcoginv[w].ReSize(3);
            // _Regcog[w].ReSize(3);
            // _Regcog[w] = 0.0001;
            // _dcog[w].ReSize(3);
        }

        _JINT[w].ReSize(3*_parameters->_internal_force_links.size(),_numdofs);
        _JINTplus[w].ReSize(_numdofs,3*_parameters->_internal_force_links.size());

        _Mint[w].ReSize(3*_parameters->_internal_force_links.size());
        _Mintinv[w].ReSize(3*_parameters->_internal_force_links.size());

        _Regint[w].ReSize(3*_parameters->_internal_force_links.size());
        _Regint[w] = 0.001;

        _dint[w].ReSize(3*_parameters->_internal_force_links.size());

    }

    if(bPrint)
        RAVELOG_INFO("Initialize waypoints status tracker.\n");

    _stable_waypoint.resize(ptraj->GetNumWaypoints(),false);

    // std::pair<dReal,dReal> prismatic_x_limit;
    // // prismatic_x_limit.first =

    TrajectoryBasePtr ftraj = RaveCreateTrajectory(GetEnv(),"");
    ftraj->Init(_esRobot->GetActiveConfigurationSpecification());

    ftraj->Clone(ptraj,0);

    int num_waypoints = ftraj->GetNumWaypoints();

    bool all_waypoints_stable;

    if(bPrint)
        RAVELOG_INFO("Elastic Strips main loop starts.\n");

    std::vector<size_t> once_stable_waypoint;
    std::vector<size_t> non_stable_waypoint;

    _badjointinds.resize(num_waypoints);

    vector<dReal> prev_error(num_waypoints,9999.0);
    vector<dReal> stepsize(num_waypoints);

    for(int i = 0; i < stepsize.size(); i++)
    {
        stepsize[i] = 0.1 * _waypoint_manip_group_map.find(i)->second.size();
    }


    for(int k = 0; k < 20; k++) // modify the configuration
    {
        if(bPrint)
            RAVELOG_INFO("Iteration: %i\n",k);

        TrajectoryBasePtr ntraj = RaveCreateTrajectory(GetEnv(),"");
        ntraj->Init(_esRobot->GetActiveConfigurationSpecification());
        std::vector< std::vector<dReal> > tmp_traj(num_waypoints, std::vector<dReal>(_numdofs,0));
        all_waypoints_stable = true;

        if(bPrint)
            RAVELOG_INFO("Find the contact consistent manipulator transform.\n");
        // Calculate the contact consistent manipulator translation
        // FindContactConsistentManipTranslation(ftraj);
        // cout<<"Decide Contact Consistent Region."<<endl;
        DecideContactConsistentTransform(ftraj);
        // cout<<"Find Nearest Contact Region."<<endl;
        int contact_contact_region_matching = FindNearestContactRegion();

        if(contact_contact_region_matching == -1)
        {
            result = PS_Failed;
            return result;
        }

        once_stable_waypoint.clear();
        non_stable_waypoint.clear();

        // for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++)
        // {
        //     std::vector<dReal> qs(_numdofs);
        //     ftraj->GetWaypoint(w,qs);
        //     _esRobot->SetActiveDOFValues(qs);
        //     cout<<"Waypoint "<<w<<":"<<endl;
        //     cout<<"left_foot: "<<_esRobot->GetLink("l_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("l_foot")->GetTransform().trans.y<<endl;
        //     cout<<"right_foot: "<<_esRobot->GetLink("r_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("r_foot")->GetTransform().trans.y<<endl;
        // }

        // getchar();

        // cout<<"Start to modify the trajectory."<<endl;
        // #pragma omp parallel for schedule(dynamic) num_threads(16)
        for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++) // for each configuration in the trjectory
        {
            RobotBasePtr robot = _esRobotVec[w%_num_robots];

            dReal magnitude, xyz_error, total_error;

            if(bPrint)
                RAVELOG_INFO("Waypoint: %i\n",w);

            // Get the initial configuration
            std::vector<dReal> qs(_numdofs); // the initial configuration
            ftraj->GetWaypoint(w,qs);

            robot->SetActiveDOFValues(qs);

            GetEnv()->UpdatePublishedBodies();

            std::vector<dReal> qs_old = qs; // store the configuration before taking step.
            // if(_parameters->balance_mode != BALANCE_NONE)
            // {
            //     balance_checker.RefreshBalanceParameters(qs);
            // }

            // reset waypoint stability state
            if(_stable_waypoint[w] == true)
            {
                // ntraj->Insert(w,qs,robot->GetActiveConfigurationSpecification());
                _stable_waypoint[w] = false;
                // continue;
            }
            else
            {
                if(bPrint)
                    RAVELOG_INFO("Waypoint %i is not stable yet.\n",w);
            }

            bool bLimit = false;
            _badjointinds.at(w).clear();

            bool bInCollision = false;


            if(bPrint)
                RAVELOG_INFO("Move configuration according to the specified constraints.\n");

            do
            {
                // Calculate each Jacobian
                qs_old = qs;
                _xy_error[w] = 0;
                _z_error[w] = 0;
                _rpy_error[w] = 0;

                std::vector<GraphHandlePtr> handles;

                // 1. Z
                // 2. XY
                UpdateZRPYandXYJacobianandStep(Transform(),w);

                // 3. Balance / COG / Update COG
                if(_parameters->balance_mode != BALANCE_NONE)
                {
                    UpdateCOGJacobianandStep(Transform(),w);
                }

                // 4. Obstacle Avoidance
                if(_parameters->bOBSTACLE_AVOIDANCE)
                {
                    UpdateOAJacobianandStep(Transform(),ftraj,w,bInCollision);
                }

                xyz_error = sqrt(pow(_xy_error[w],2) + pow(_z_error[w],2));

                //may require another condition to check posture control
                if((sqrt(pow(xyz_error,2) + pow(_rpy_error[w],2)) < epsilon) &&
                   (_parameters->balance_mode == BALANCE_NONE || _balance_checkers[w]->CheckSupport(_curcogs[w])) &&
                   (_parameters->bOBSTACLE_AVOIDANCE == false || bInCollision == false))
                {
                    _stable_waypoint[w] = true;
                    if(bPrint)
                        RAVELOG_INFO("Waypoint %i is stable.\n",w);
                    break;
                }
                else
                {
                    if(bPrint)
                    // if(true)
                    {
                        cout<<"Waypoint: "<<w<<endl;
                        cout<<"Reach Point: "<<(sqrt(pow(xyz_error,2) + pow(_rpy_error[w],2)) < epsilon)<<endl;
                        cout<<"balanced: "<<(_parameters->balance_mode == BALANCE_NONE || _balance_checkers[w]->CheckSupport(_curcogs[w]))<<endl;
                        cout<<"free of collision: "<<(_parameters->bOBSTACLE_AVOIDANCE == false || bInCollision == false)<<endl;

                        cout<<"xy_error: "<<_xy_error[w]<<endl;
                        cout<<"z_error: "<<_z_error[w]<<endl;
                        cout<<"rpy_error: "<<_rpy_error[w]<<endl;
                        cout<<"cog_error: "<< (_curcogs[w]-_cogtargs[w]).lengthsqr2()<<endl;

                        cout<<"curcog: "<<_curcogs[w].x<<", "<<_curcogs[w].y<<", "<<_curcogs[w].z<<endl;
                        cout<<"cogtarg: "<<_cogtargs[w].x<<", "<<_cogtargs[w].y<<", "<<_cogtargs[w].z<<endl;

                        cout<<"Link in collision: ";
                        for(std::set<string>::iterator link_it = _links_in_collision[w].begin(); link_it != _links_in_collision[w].end(); link_it++)
                        {
                            cout<<(*link_it)<<" ";
                        }
                        cout<<endl;

                        cout<<"Joint in limit: ";
                        for(std::vector<int>::iterator it = _badjointinds.at(w).begin(); it != _badjointinds.at(w).end(); it++)
                        {
                            cout<<robot->GetJointFromDOFIndex(robot->GetActiveDOFIndices()[*it])->GetName()<<" ";
                        }
                        cout<<endl;

                        // if(!_links_in_collision[w].empty())
                        // {
                        //     getchar();
                        // }

                        // if(_parameters->balance_mode != BALANCE_NONE)
                        // {
                        //     getchar();
                        // }
                        // getchar();
                    }

                    // if(k > 5)
                    // {
                    //     getchar();
                    // }
                }

                // 5. Posture Control (relative to robot base)
                if(_parameters->bPOSTURE_CONTROL)
                {
                    UpdatePCJacobianandStep(Transform(),w);
                }

                UpdateINTJacobianandStep(Transform(),ftraj,w);

                // Calculate steps
                if(_parameters->balance_mode != BALANCE_NONE)
                {
                    total_error = sqrt(pow(xyz_error,2) + pow(_rpy_error[w],2) + pow(_dcog[w](1)/_ccog,2) + pow(_dcog[w](2)/_ccog,2));
                }
                else
                {
                    total_error = sqrt(pow(xyz_error,2) + pow(_rpy_error[w],2));
                }

                if(total_error > stepsize[w])
                    magnitude = stepsize[w]/total_error;
                else
                    magnitude = 1.0;

                // NEWMAT::ColumnVector tempstep;
                // tempstep.ReSize(_numdofs);
                _step[w] = 0;
                _balance_step[w] = 0;
                _oa_step[w] = 0;
                _zrpy_step[w] = 0;
                _xy_step[w] = 0;
                _pc_step[w] = 0;
                _int_step[w] = 0;

                string highest_priority = "None";

                _int_step[w] = _JINTplus[w] * _dint[w];
                _JHP[w] = &_JINT[w];
                _JHPplus[w] = &_JINTplus[w];
                _dhp[w] = &_dint[w];

                if(_parameters->bPOSTURE_CONTROL)
                {
                    _pc_step[w] = _JPCplus[w] * _dpc[w] + (NEWMAT::IdentityMatrix(_numdofs) - _JPCplus[w]*_JPC[w]) * _int_step[w];
                    highest_priority = "PC";
                    _JHP[w] = &_JPC[w];
                    _JHPplus[w] = &_JPCplus[w];
                    _dhp[w] = &_dpc[w];
                }
                else
                {
                    _pc_step[w] = _int_step[w];
                }

                // if(_parameters->balance_mode != BALANCE_NONE)
                // {
                //     balance_step = JCOGplus*dcog + (NEWMAT::IdentityMatrix(_numdofs) - JCOGplus*JCOG) * pc_step;
                //     highest_priority = "BALANCE";
                //     JHP = &JCOG;
                //     JHPplus = &JCOGplus;
                //     dhp = &dcog;
                // }
                // else
                // {
                //     balance_step = pc_step;
                // }

                // if(_parameters->bOBSTACLE_AVOIDANCE)
                // {
                //     oa_step = JOAplus*doa + (NEWMAT::IdentityMatrix(_numdofs) - JOAplus*JOA) * balance_step;
                //     highest_priority = "OA";
                //     // JHP = &JOA;
                //     // JHPplus = &JOAplus;
                //     // dhp = &doa;
                // }
                // else
                // {
                //     oa_step = pc_step;
                // }

                NEWMAT::Matrix JM;
                NEWMAT::Matrix JMplus;
                NEWMAT::ColumnVector dm;
                NEWMAT::SymmetricMatrix Mm;
                NEWMAT::SymmetricMatrix Mminv;
                NEWMAT::DiagonalMatrix Regm;
                NEWMAT::ColumnVector m_step;

                // if(strcmp(highest_priority.c_str(),"None") == 0)
                if(_parameters->balance_mode == BALANCE_NONE)
                {
                    JM = _JXYZRPY[w];
                    JMplus = _JXYZRPYplus[w];
                    dm = _dxyzrpy[w];
                }
                else
                {
                    // for(int i = 0; i < _numdofs; i++)
                    // {
                    //     if(fabs(qs[i] - _lowerLimit[i]) < 0.0001 || fabs(qs[i] - _upperLimit[i]) < 0.0001)
                    //     {
                    //         W(i+1) = 0.25 * (_upperLimit[i]-_lowerLimit[i]) / 0.0001;
                    //     }
                    //     else
                    //     {
                    //         W(i+1) = 0.25 * pow((_upperLimit[i]-_lowerLimit[i]),2) / ((qs[i] - _lowerLimit[i]) * (_upperLimit[i]-qs[i]));
                    //     }

                    //     Winv(i+1) = 1 / W(i+1);
                    // }

                    JM.ReSize(0,_numdofs);
                    JM &= _JXYZRPY[w];
                    JM &= _JCOG[w];

                    dm.ReSize(0);
                    dm &= _dxyzrpy[w];
                    dm &= _dcog[w];

                    JMplus.ReSize(_numdofs,JM.Nrows());
                    Mm.ReSize(JM.Nrows());
                    Mminv.ReSize(JM.Nrows());
                    Regm.ReSize(JM.Nrows());
                    Regm = 0.001;

                    Mm << (JM*JM.t()) + Regm;
                    invConditioningBound(10000,Mm,Mminv);
                    JMplus = JM.t()*Mminv;

                    // Mm << (JM*Winv*JM.t()) + Regm;
                    // invConditioningBound(10000,Mm,Mminv);
                    // JMplus = Winv*JM.t()*Mminv;
                }

                // m_step = JMplus*dm;

                if(_parameters->bOBSTACLE_AVOIDANCE)
                {
                    m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * (_JOAplus[w]*_doa[w] + (NEWMAT::IdentityMatrix(_numdofs) - _JOAplus[w]*_JOA[w]) * _int_step[w]);
                }
                else
                {
                    m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * _int_step[w];
                    // m_step = JMplus*dm;
                }

                // if(strcmp(highest_priority.c_str(),"None") == 0 or strcmp(highest_priority.c_str(),"PC") == 0)
                // {
                //     m_step = JMplus*dm;
                // }
                // else if(strcmp(highest_priority.c_str(),"BALANCE") == 0)
                // {
                //     m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * pc_step;
                // }
                // else if(strcmp(highest_priority.c_str(),"OA") == 0)
                // {
                //     m_step = JMplus*dm + (NEWMAT::IdentityMatrix(_numdofs) - JMplus*JM) * balance_step;
                // }

                // step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*m_step;
                // step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*(JXYplus*dxy);
                // step = JZRPYplus * dzrpy + (NEWMAT::IdentityMatrix(_numdofs) - JZRPYplus*JZRPY)*(JXYplus*dxy + (NEWMAT::IdentityMatrix(_numdofs) - JXYplus*JXY)*(JCOGplus*dcog));

                // step = JXYZRPYplus * dxyzrpy;

                _step[w] = magnitude * m_step;

                // if(k > 0 && step.Sum() < 0.001)
                // {
                //     result = PS_Failed;

                //     return result;
                // }

                //add step and check for joint limits
                bLimit = false;

                for(int i = 0; i < _numdofs; i++)
                {
                    qs[i] = qs_old[i] - _step[w](i+1);
                    if(qs[i] < _lowerLimit[i] || qs[i] > _upperLimit[i])
                    {
                        if(bPrint)
                        {
                            RAVELOG_INFO("Joint: %s, value: %f, step: %f, limit: %f <=> %f\n",robot->GetJointFromDOFIndex(robot->GetActiveDOFIndices()[i])->GetName().c_str(),qs_old[i],_step[w](i+1),_lowerLimit[i],_upperLimit[i]);

                            cout<<"over joint limit: "<<i<<endl;
                            cout<<qs[i]<<", limit: ("<<_lowerLimit[i]<<","<<_upperLimit[i]<<")"<<endl;
                        }

                        if(qs[i] < _lowerLimit[i])
                            qs[i] = _lowerLimit[i];
                        if(qs[i] > _upperLimit[i])
                            qs[i] = _upperLimit[i];

                        _badjointinds.at(w).push_back(i); //note this will never add the same joint twice

                        bLimit = true;

                    }
                }

                if(bLimit)
                {
                    // cout<<"***************over joint limit.*****************"<<endl;
                    qs = qs_old;
                }
                // else
                // {
                //     /***********************/
                //     map<string,int> manip_group_map = waypoint_manip_group_map.find(w)->second;

                //     for(map<string,int>::iterator mgm_it = manip_group_map.begin(); mgm_it != manip_group_map.end(); mgm_it++)
                //     {
                //         string manip_name = mgm_it->first;
                //         int manip_group_index = mgm_it->second;

                //         Transform contact_consistent_transform = contact_manips_group.find(manip_group_index)->second.contact_consistent_transform;

                //         Vector cct_origin = contact_consistent_transform.trans;
                //         TransformMatrix contact_consistent_transform_matrix = TransformMatrix(contact_consistent_transform);
                //         Vector cct_x = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[0],contact_consistent_transform_matrix.m[4],contact_consistent_transform_matrix.m[8]);
                //         Vector cct_y = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[1],contact_consistent_transform_matrix.m[5],contact_consistent_transform_matrix.m[9]);
                //         Vector cct_z = contact_consistent_transform.trans + 0.5 * Vector(contact_consistent_transform_matrix.m[2],contact_consistent_transform_matrix.m[6],contact_consistent_transform_matrix.m[10]);


                //         // handles.push_back(_esEnv->drawarrow(cct_origin, cct_x, 0.008, RaveVector<float>(1, 0, 0, 1)));
                //         // handles.push_back(_esEnv->drawarrow(cct_origin, cct_y, 0.008, RaveVector<float>(0, 1, 0, 1)));
                //         // handles.push_back(_esEnv->drawarrow(cct_origin, cct_z, 0.008, RaveVector<float>(0, 0, 1, 1)));

                //         robot->SetActiveDOFValues(qs_old);


                //         RobotBase::ManipulatorPtr target_manip = robot->GetManipulators()[GetManipIndex.find(manip_name)->second];

                //         Vector from_vec = target_manip->GetTransform().trans;

                //         robot->SetActiveDOFValues(qs);

                //         Vector to_vec = target_manip->GetTransform().trans;
                //         to_vec.x = 20*(to_vec.x - from_vec.x) + from_vec.x;
                //         to_vec.y = 20*(to_vec.y - from_vec.y) + from_vec.y;
                //         to_vec.z = 20*(to_vec.z - from_vec.z) + from_vec.z;


                //         // handles.push_back(_esEnv->drawarrow(from_vec, to_vec, 0.012, RaveVector<float>(1, 1, 0, 1)));

                //         // NEWMAT::ColumnVector step_zrpy = JZRPYplus * dzrpy;
                //         // vector<dReal> qs_2(_numdofs);

                //         // for(int i = 0; i < _numdofs; i++)
                //         // {
                //         //     qs_2[i] = qs_old[i] - magnitude * step_zrpy(i+1);
                //         // }

                //         // robot->SetActiveDOFValues(qs_2);

                //         // Vector to_vec_2 = target_manip->GetTransform().trans;
                //         // to_vec_2.x = 20*(to_vec_2.x - from_vec.x) + from_vec.x;
                //         // to_vec_2.y = 20*(to_vec_2.y - from_vec.y) + from_vec.y;
                //         // to_vec_2.z = 20*(to_vec_2.z - from_vec.z) + from_vec.z;

                //         // handles.push_back(_esEnv->drawarrow(from_vec, to_vec_2, 0.012, RaveVector<float>(0, 1, 1, 1)));

                //         cout<<manip_name<<": ("<<to_vec.x-from_vec.x<<","<<to_vec.y-from_vec.y<<","<<to_vec.z-from_vec.z<<")"<<endl;

                //         robot->SetActiveDOFValues(qs_old);

                //     }

                //     getchar();
                //     /***********************/
                // }

            }while(bLimit);

            if(bPrint)
                RAVELOG_INFO("Move a step.\n");

            if(total_error >= prev_error[w] && !_stable_waypoint[w])
            {
                stepsize[w] = stepsize[w]/2;
                // xyz_error = prev_error[w];
                // qs = qs_old;
            }
            // else
            // {
            //     stepsize[w] = 0.1 * _waypoint_manip_group_map.find(w)->second.size();
            // }

            prev_error[w] = total_error;

            // if(sqrt(pow(xyz_error,2) + pow(rpy_error,2)) >= prev_error[w] && !stable_waypoint.find(w)->second)
            // {
            //     stepsize[w] = stepsize[w]/2;
            //     // xyz_error = prev_error[w];
            //     // qs = qs_old;
            // }
            // else if(sqrt(pow(xyz_error,2) + pow(rpy_error,2)) < prev_error[w] && !stable_waypoint.find(w)->second)
            // {
            //     stepsize[w] = 0.1 * waypoint_manip_group_map.find(w)->second.size();
            // }

            // prev_error[w] = sqrt(pow(xyz_error,2) + pow(rpy_error,2));

            // for(int i = 0; i < qs.size(); i++)
            // {
            //     cout<<qs[i]<<' ';
            // }
            // cout<<endl;

            // robot->SetActiveDOFValues(qs);

            // GetEnv()->UpdatePublishedBodies();

            tmp_traj[w] = qs;

            // getchar();

            if(bPrint)
                RAVELOG_INFO("=================================================\n");
        }

        for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++)
        {
            if(_stable_waypoint[w] == true)
            {
                once_stable_waypoint.push_back(w);
            }
            else
            {
                non_stable_waypoint.push_back(w);
                all_waypoints_stable = false;
            }
            ntraj->Insert(w,tmp_traj[w],_esRobot->GetActiveConfigurationSpecification());
        }

        // cout<<"Number of contact consistent positions: "<<contact_consistent_manip_translation.size()<<endl;
        // for(std::vector< std::pair<string,Vector> >::iterator ccmt_it = contact_consistent_manip_translation.begin(); ccmt_it != contact_consistent_manip_translation.end(); ccmt_it++)
        // {
        //     cout<<"contact "<<ccmt_it-contact_consistent_manip_translation.begin()<<", "<<ccmt_it->first<<": "<<ccmt_it->second.x<<", "<<ccmt_it->second.y<<", "<<ccmt_it->second.z<<endl;
        // }

        // cout<<"Total Waypoints: "<<ftraj->GetNumWaypoints()<<", Once stable waypoints: ";
        // for(vector<size_t>::iterator osw_it = once_stable_waypoint.begin(); osw_it != once_stable_waypoint.end(); osw_it++)
        // {
        //     cout<<*osw_it<<' ';
        // }
        // cout<<endl;


        if(bPrint)
        {
            cout<<"Total Waypoints: "<<ftraj->GetNumWaypoints()<<", Non-stable Waypoint Num: "<< non_stable_waypoint.size() <<", Non-stable waypoints: ";
            for(vector<size_t>::iterator nsw_it = non_stable_waypoint.begin(); nsw_it != non_stable_waypoint.end(); nsw_it++)
            {
                cout << (*nsw_it) << ' ';
            }
            cout << endl;

            // cout<<"Total Waypoints: "<<ftraj->GetNumWaypoints()<<", Non-stable waypoints: ";
            // for(size_t waypoint_i = 0; waypoint_i < ftraj->GetNumWaypoints(); waypoint_i++)
            // {
            //     for(vector<size_t>::iterator osw_it = once_stable_waypoint.begin(); osw_it != once_stable_waypoint.end(); osw_it++)
            //     {
            //         if(*osw_it == waypoint_i)
            //         {
            //             break;
            //         }
            //         else if(*osw_it > waypoint_i)
            //         {
            //             cout << waypoint_i << ' ';
            //             break;
            //         }

            //     }
            // }
            // cout<<endl;
        }

        // for(unsigned int w = 0; w < ftraj->GetNumWaypoints(); w++)
        // {
        //     std::vector<dReal> qs(_numdofs);
        //     ftraj->GetWaypoint(w,qs);
        //     _esRobot->SetActiveDOFValues(qs);
        //     cout<<"After iteration: "<<k<<", Waypoint "<<w<<":"<<endl;
        //     cout<<"left_foot: "<<_esRobot->GetLink("l_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("l_foot")->GetTransform().trans.y<<endl;
        //     cout<<"right_foot: "<<_esRobot->GetLink("r_foot")->GetTransform().trans.x<<", "<<_esRobot->GetLink("r_foot")->GetTransform().trans.y<<endl;
        // }

        // getchar();

        ftraj->Clone(ntraj,0);

        if(all_waypoints_stable)
        {
            ptraj->Clone(ftraj,0);
            if(bPrint)
                RAVELOG_INFO("Iteration used: %i\n",k);
            return result;
        }

    }

    ptraj->Clone(ftraj,0);

    result = PS_Failed;

    return result;
}

void ElasticStrips::QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi)
{
    //used in QuatToRPY
    dReal a,b,c,d;
    Vector RPYIdentityOffsets[8] = { Vector(M_PI,M_PI,M_PI),
                                    Vector(M_PI,M_PI,-M_PI),
                                    Vector(M_PI,-M_PI,M_PI),
                                    Vector(M_PI,-M_PI,-M_PI),
                                    Vector(-M_PI,M_PI,M_PI),
                                    Vector(-M_PI,M_PI,-M_PI),
                                    Vector(-M_PI,-M_PI,M_PI),
                                    Vector(-M_PI,-M_PI,-M_PI)};
    dReal min_dist;
    Vector temp_vec;
    dReal temp_dist;

    a = tm.rot.x;
    b = tm.rot.y;
    c = tm.rot.z;
    d = tm.rot.w;

    //psi theta and phi will always be between -pi and pi
    psi = atan2(2*a*b + 2*c*d, a*a - b*b - c*c + d*d); //psi
    theta = -asin(2*b*d-2*a*c); //theta
    phi = atan2(2*a*d+2*b*c, a*a + b*b - c*c - d*d); //phi


    //go through all the identities and find which one minimizes the total rotational distance
    //don't need to consider +/-2pi b/c all three angles are between -pi and pi
    min_dist = 10000;
    for(int i =0; i < 9; i++)
    {

        if(i == 0) //for first element, use original values
        {
            temp_vec.x = psi;
            temp_vec.y = theta;
            temp_vec.z = phi;
        }
        else
        {
            temp_vec.x = psi + RPYIdentityOffsets[i-1].x;
            temp_vec.y = -theta + RPYIdentityOffsets[i-1].y;//note that theta is negative
            temp_vec.z = phi + RPYIdentityOffsets[i-1].z;
        }

        temp_dist = temp_vec.lengthsqr3();
        if(temp_dist < min_dist)
        {
            min_dist = temp_dist;
            psi = temp_vec.x;
            theta = temp_vec.y;
            phi = temp_vec.z;
        }
    }

    //RAVELOG_INFO("psi: %f, theta: %f, phi: %f\n",psi,theta,phi);
}

int ElasticStrips::invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{
    //for inverse conditioning
    NEWMAT::DiagonalMatrix S;
    NEWMAT::Matrix V;

    int didfix = 0;
    NEWMAT::EigenValues(A,S,V);
    // Find the maximum eigenvalue
    dReal maxEig = 0;
    dReal minEig = 0;
    for (int i = 1; i <= S.Nrows(); ++i){
        dReal e = S(i);
        if (e > maxEig) maxEig = e;
        if (i == 1 || e < minEig) minEig = e;
    }
    //RAVELOG_INFO("min/max eigenvalue: %f/%f\n", minEig, maxEig);

    dReal minEigDesired = maxEig/maxConditionNumber;
    int notfixcount = 0;
    for (int i = 1; i <= S.Nrows(); ++i){
        dReal e = S(i);
        if (e < minEigDesired) {e = minEigDesired; didfix = 1;}
        else
            notfixcount++;

        if (maxEig > 100) { e = e/maxEig*100;}
        S(i) = e;
    }


    //this will do the inversion
    Afixed << V * S.i() * V.t();
    return didfix;
}

void ElasticStrips::PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement)
{
    //return;

    stringstream s;
    s.setf(ios::fixed,ios::floatfield);
    s.precision(5);
    s << "\n";
    if(statement != NULL)
        s << statement <<"\n";
    for(int i = 0; i < numrows; i++)
    {
        for(int j = 0; j < numcols; j++)
        {
            s << pMatrix[i*numcols + j] << " \t";
            if(fabs(pMatrix[i*numcols + j]) > 10000)
                RAVELOG_INFO("WARNING: %d %d is huge!!!\n",i,j);
        }
        s << endl;
    }
    RAVELOG_INFO(s.str().c_str());

}
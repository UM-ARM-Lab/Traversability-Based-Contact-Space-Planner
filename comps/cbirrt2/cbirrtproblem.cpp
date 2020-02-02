/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Dmitry Berenson <dberenso@cs.cmu.edu>

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
/** \file cbirrtproblem.cpp
    \brief Implements the cbirrt problem class.
 */
#include "stdafx.h"

std::vector<GraphHandlePtr> graphptrs;

CBirrtProblem::CBirrtProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = ":Interface Author: Dmitry Berenson\nInterface to CBiRRT planner that parses input and passes it to the planner. Can also call the GeneralIK plugin. \n\n`C++ Documentation <http://automation.berkeley.edu/~berenson/docs/cbirrt/index.html>`_";
    RegisterCommand("GrabBody",boost::bind(&CBirrtProblem::GrabBody,this,_1,_2),
                    "Robot calls ::Grab on a body with its current manipulator");

    RegisterCommand("RunCBirrt",boost::bind(&CBirrtProblem::RunCBirrt,this,_1,_2),
                    "Run the CBirrt Planner");

    RegisterCommand("DoGeneralIK",boost::bind(&CBirrtProblem::DoGeneralIK,this,_1,_2),
                    "Calls the General IK solver");

    RegisterCommand("CheckSupport",boost::bind(&CBirrtProblem::CheckSupport,this,_1,_2),
                    "Checks whether the cg of the robot is over the given support polygon");

    RegisterCommand("CheckSelfCollision",boost::bind(&CBirrtProblem::CheckSelfCollision,this,_1,_2),
                    "Checks whether the robot is in self-collision");

    RegisterCommand("GetJointAxis",boost::bind(&CBirrtProblem::GetJointAxis,this,_1,_2),
                    "Returns the joint axis of a given kinbody");

    RegisterCommand("GetJointTransform",boost::bind(&CBirrtProblem::GetJointTransform,this,_1,_2),
                    "Returns the joint transform of a given kinbody");

    RegisterCommand("GetCamView",boost::bind(&CBirrtProblem::GetCamView,this,_1,_2),
                    "Get the currnet camera view");

    RegisterCommand("SetCamView",boost::bind(&CBirrtProblem::SetCamView,this,_1,_2),
                    "Set the currnet camera view");

    RegisterCommand("Traj",boost::bind(&CBirrtProblem::Traj,this,_1,_2),
                    "Execute a trajectory from a file on the local filesystem");

    RegisterCommand("GetPlannerState",boost::bind(&CBirrtProblem::GetPlannerState,this,_1,_2),
                    "Returns the current state of the planner");

    RegisterCommand("StopPlanner",boost::bind(&CBirrtProblem::StopPlanner,this,_1,_2),
                    "Stops the planner if it is running");

    RegisterCommand("ClearDrawn",boost::bind(&CBirrtProblem::ClearDrawn,this,_1,_2),
                    "Clears objects drawn by cbirrt planner and problem");


    _reusePlanner = false;
    _plannerState = PS_Idle;
    _plannerThread.reset();

}

void CBirrtProblem::Destroy()
{

}

CBirrtProblem::~CBirrtProblem()
{
    Destroy();
}

int CBirrtProblem::main(const std::string& cmd)
{

    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
       _strRobotName = p;

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots);

    _pIkSolver = RaveCreateIkSolver(GetEnv(),"GeneralIK");
    _pIkSolver->Init(robot->GetActiveManipulator());
    RAVELOG_INFO("IKsolver initialized\n");
    
    return 0;
}

void CBirrtProblem::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
{


    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( strcmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}


bool CBirrtProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

bool CBirrtProblem::ClearDrawn(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    graphptrs.clear();
    return true;
}


int CBirrtProblem::CheckSelfCollision(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    if(robot->CheckSelfCollision())
        sout << "1";
    else
        sout << "0";
    
    return true;
}

//! TEMP
void draw_tf(EnvironmentBasePtr env, const Transform &tf, const Transform &origin_tf = Transform());

bool CBirrtProblem::DoGeneralIK(ostream& sout, istream& sinput)
{
    // Lock environment mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    RAVELOG_DEBUG("Starting General IK...\n");
    // RAVELOG_INFO("Checking ravelog_info\n");

    // Parameters to populate from sinput
    dReal temp_r;                   // Used in nummanips
    bool bExecute = false;             // from exec
    bool bGetTime = false;             // from gettime
    bool bNoRot = false;               // from norot
    bool bReturnClosest = false;       // from returnclosest
    bool bObstacleAvoidance = false;   // from obstacle avoidance
    bool bCOG = false;                 // from balance
    bool bReuseGIWC = false;           // from reusegiwc
    bool bExactCOG = false;            // from exact_com
    TransformMatrix robot_tf;          // from robottm
    TransformMatrix temp_tf_matrix; // Used in maniptm
    Transform temp_tf;              // Used in maniptm
    std::vector<int> checkcollisionlink_indices;  // from checkcollisionlink
    std::vector<int> checkselfcollisionlinkpair_indices;  // from checkselfcollisionlinkpair
    std::vector<int> obstacle_indices;  // from obstacles
    std::vector< Vector > obstacle_repulsive_vector;
    Vector cogtarg;                     // from movecog
    int balance_mode = 0;               // from movecog
    std::vector<string> supportlinks;   // from supportlinks
    Vector polyscale(1.0, 1.0, 1.0);    // from polyscale
    Vector polytrans(0, 0, 0);          // from polytrans
    std::vector<string> support_manips; // from support
    std::vector<dReal> support_mus;     // from support
    Vector gravity(0.0, 0.0, -9.8);     // from gravity

    // Populated from info in supportlinks
    std::vector<dReal> supportpolyx;
    std::vector<dReal> supportpolyy;

    // Actual parameter vector directly passed to GeneralIK
    std::vector<dReal> ikparams;

    // Command string holds the current command
    string cmd;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "nummanips") == 0 ) {
            // Number of manipulators whose transforms are specified.
            // Seems like this always agrees with the number of "maniptm" commands?
            sinput >> temp_r; ikparams.push_back(temp_r);
        }
        else if( stricmp(cmd.c_str(), "exec") == 0 ) {
            // Whether to execute the produced trajectory
            bExecute = true;
        }
        else if( stricmp(cmd.c_str(), "gettime") == 0 ) {
            // Whether the command's output should include the time.
            // If included it appears as the last space-separated number in the output string.
            bGetTime = true;
        }
        else if( stricmp(cmd.c_str(), "norot") == 0 ) {
            // TODO: Figure out the purpose of this. It's used to set the planning dof in GeneralIK.
            bNoRot = true;
        }
        else if( stricmp(cmd.c_str(), "returnclosest") == 0 ) {
            // If no solution was found, whether to return the closest solution instead of reporting failure.
            bReturnClosest = true;
        }
        else if( stricmp(cmd.c_str(), "robottm") == 0 ) {
            // Desired robot transform. It seems this is equivalent to calling robot->SetTransform() before DoGeneralIK.
            sinput >> robot_tf.m[0];
            sinput >> robot_tf.m[4];
            sinput >> robot_tf.m[8];
            sinput >> robot_tf.m[1];
            sinput >> robot_tf.m[5];
            sinput >> robot_tf.m[9];
            sinput >> robot_tf.m[2];
            sinput >> robot_tf.m[6];
            sinput >> robot_tf.m[10];
            sinput >> robot_tf.trans.x;
            sinput >> robot_tf.trans.y;
            sinput >> robot_tf.trans.z;
            robot->SetTransform(robot_tf);
        }
        else if( stricmp(cmd.c_str(), "maniptm") == 0 ) {
            // Desired transform for the given manipulator. May be specified more than once with different manipulators.
            sinput >> temp_r;
            ikparams.push_back(temp_r);

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

            ikparams.push_back(temp_tf.rot.x); ikparams.push_back(temp_tf.rot.y); ikparams.push_back(temp_tf.rot.z); ikparams.push_back(temp_tf.rot.w);
            ikparams.push_back(temp_tf.trans.x); ikparams.push_back(temp_tf.trans.y); ikparams.push_back(temp_tf.trans.z);



        }
        else if( stricmp(cmd.c_str(), "checkcollision") == 0 ) {
            bObstacleAvoidance = true;
            int numcheckcollisionlinks;
            string tempstring;
            sinput >> numcheckcollisionlinks;
            for(int i = 0; i < numcheckcollisionlinks; i++)
            {
                sinput >> tempstring;
                checkcollisionlink_indices.push_back(robot->GetLink(tempstring)->GetIndex());
            }
            std::sort(checkcollisionlink_indices.begin(),checkcollisionlink_indices.end());
        }
        else if( stricmp(cmd.c_str(), "checkselfcollision") == 0 ) {
            bObstacleAvoidance = true;
            int numcheckselfcollisionlinkpairs;
            string tempstring_1;
            string tempstring_2;
            sinput >> numcheckselfcollisionlinkpairs;
            for(int i = 0; i < numcheckselfcollisionlinkpairs; i++)
            {
                sinput >> tempstring_1;
                sinput >> tempstring_2;
                checkselfcollisionlinkpair_indices.push_back(robot->GetLink(tempstring_1)->GetIndex());
                checkselfcollisionlinkpair_indices.push_back(robot->GetLink(tempstring_2)->GetIndex());
            }
        }
        else if( stricmp(cmd.c_str(), "obstacles") == 0 ) {
            int numobstacles;
            string tempstring;
            sinput >> numobstacles;
            for(int i = 0; i < numobstacles; i++)
            {
                sinput >> tempstring;
                std::vector<KinBodyPtr> bodies;
                GetEnv()->GetBodies(bodies);
                for(int j = 0; j < bodies.size(); j++)
                {
                    if(stricmp(tempstring.c_str(),bodies[j]->GetName().c_str()) == 0)
                    {
                        obstacle_indices.push_back(j);
                        break;
                    }
                }
                Vector repulsive_vector;

                sinput >> repulsive_vector.x;
                sinput >> repulsive_vector.y;
                sinput >> repulsive_vector.z;

                obstacle_repulsive_vector.push_back(repulsive_vector);
            }
            // std::sort(obstacle_indices.begin(),obstacle_indices.end());
        }
        else if( stricmp(cmd.c_str(), "movecog") == 0 ) {
            // TODO: Verify this description
            // This appears to the desired Center of Gravity position. Note that support-polygon stability checking is
            // not performed if this command is not provided.
            bCOG = true;
            sinput >> cogtarg.x;
            sinput >> cogtarg.y;
            sinput >> cogtarg.z;
        }
        else if(stricmp(cmd.c_str(), "supportlinks") == 0 ){
            // The links to use for support-polygon stability checking. Format is
            // "supportlinks {num_links} {link_name}..."
            int numsupportlinks;
            string tempstring;
            sinput >> numsupportlinks;
            for(int i =0; i < numsupportlinks; i++)
            {
                sinput >> tempstring;
                supportlinks.push_back(tempstring);
            }
            balance_mode = 1;
        }
        else if(stricmp(cmd.c_str(), "polyscale") == 0 ){
            // 3d vector to scale the support polygon for support-polygon stability checking. Intended to make it
            // possible to specify more conservative constraints (disallow barely-balanced situations).
            sinput >> polyscale.x;
            sinput >> polyscale.y;
            sinput >> polyscale.z;
        }
        else if(stricmp(cmd.c_str(), "polytrans") == 0 ){
            // 3d vector to translate the support polygon for support-polygon stability checking.
            sinput >> polytrans.x;
            sinput >> polytrans.y;
            sinput >> polytrans.z;
        }
        else if (stricmp(cmd.c_str(), "support") == 0){
            // Support specifier for GIWC stability checking. May be provided more than once. Format is
            // "support {manip_index} {friction_coeff}".
            string tempstr;
            sinput >> tempstr; support_manips.push_back(tempstr);
            sinput >> temp_r; support_mus.push_back(temp_r);
            balance_mode = 2;
        }
        else if (stricmp(cmd.c_str(), "gravity") == 0){
            sinput >> gravity.x;
            sinput >> gravity.y;
            sinput >> gravity.z;
        }
        else if (stricmp(cmd.c_str(), "reusegiwc") == 0){
            bReuseGIWC = true;
        }
        else if (stricmp(cmd.c_str(), "exactcom") == 0){
            bExactCOG = true;
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if(bObstacleAvoidance)
    {
        ikparams.push_back(1); //the robot should avoid collision, including self collision
        
        ikparams.push_back(checkcollisionlink_indices.size());
        for(std::vector<int>::iterator it = checkcollisionlink_indices.begin(); it != checkcollisionlink_indices.end(); it++)
        {
            ikparams.push_back(*it);
        }

        ikparams.push_back((checkselfcollisionlinkpair_indices.size()/2));
        for(std::vector<int>::iterator it = checkselfcollisionlinkpair_indices.begin(); it != checkselfcollisionlinkpair_indices.end(); it++)
        {
            ikparams.push_back(*it);
        }

        ikparams.push_back(obstacle_indices.size());
        for(int i = 0; i < obstacle_indices.size(); i++)
        {
            ikparams.push_back(obstacle_indices[i]);
            ikparams.push_back(obstacle_repulsive_vector[i].x);
            ikparams.push_back(obstacle_repulsive_vector[i].y);
            ikparams.push_back(obstacle_repulsive_vector[i].z);
        }
    }
    else
    {
        ikparams.push_back(0); //the robot need not avoid collision
    }

    ikparams.push_back(balance_mode);
    if(balance_mode == 1)
    {
        ikparams.push_back(cogtarg.x);
        ikparams.push_back(cogtarg.y);
        ikparams.push_back(cogtarg.z);

        if(supportlinks.size() == 0)
        {
            RAVELOG_INFO("ERROR: Must specify support links to do balancing\n");
            return false;
        }

        GetSupportPolygon(supportlinks, supportpolyx, supportpolyy, polyscale, polytrans);

        ikparams.push_back(supportpolyx.size());

        for(int i = 0 ; i < supportpolyx.size(); i ++)
            ikparams.push_back(supportpolyx[i]);

        for(int i = 0 ; i < supportpolyy.size(); i ++)
            ikparams.push_back(supportpolyy[i]);
    }
    else if(balance_mode == 2)
    {
        ikparams.push_back(gravity.x);
        ikparams.push_back(gravity.y);
        ikparams.push_back(gravity.z);

        // Compute cogtarg as the mean position of the support links
        if( !bCOG )
        {
            float total_weight = 0.0;
            cogtarg = Vector(0,0,0);
            for(int i = 0; i < support_manips.size(); i++) {
                Transform tf = robot->GetManipulator(support_manips[i])->GetTransform();
                if(strcmp(support_manips[i].c_str(), "l_leg") == 0 || strcmp(support_manips[i].c_str(), "r_leg") == 0)
                {
                    cogtarg += tf.trans;
                    total_weight += 1.0;
                }
                else
                {
                    cogtarg += tf.trans;
                    total_weight += 1.0;
                    // total_weight += 0.0;
                }
            }
            // cogtarg /= support_manips.size();
            cogtarg /= total_weight;
            // cogtarg.z = cogtarg.z + 0.8;

            ikparams.push_back(0);
        }
        else
        {
            ikparams.push_back(0);
        }
        ikparams.push_back(cogtarg.x);
        ikparams.push_back(cogtarg.y);
        ikparams.push_back(cogtarg.z);

        
        bool same_support_manips = true;
        if(support_manips.size() == prev_support_manips.size())
        {
            for(int i = 0; i < support_manips.size(); i++)
            {
                if(strcmp(support_manips[i].c_str(), prev_support_manips[i].c_str()) != 0)
                {
                    same_support_manips = false;
                    break;
                }
            }
        }
        else
        {
            same_support_manips = false;
        }

        unsigned long before_GIWC = timeGetTime();

        if(same_support_manips && bReuseGIWC)
        {
            // ikparams.reserve(ikparams.size() + distance(prev_giwc.begin(),prev_giwc.end()));
            ikparams.insert(ikparams.end(),prev_giwc.begin(),prev_giwc.end());
        }
        else
        {
            GetGIWC(support_manips, support_mus, ikparams);
        }

        if(bReuseGIWC)
        {
            prev_support_manips = support_manips;
        }
        else
        {
            prev_support_manips.clear();
        }        

        int GIWC_timetaken = timeGetTime() - before_GIWC;

        // RAVELOG_INFO("GIWC Got! (%d ms)\n",GIWC_timetaken);
    }


    std::vector<dReal> q0(robot->GetActiveDOF());
    robot->GetActiveDOFValues(q0);
    
    std::vector<dReal> qResult(robot->GetActiveDOF());

    ikparams.push_back(0);//select the mode
    if(bNoRot)
        ikparams.push_back(1);//don't do rotation
    else
        ikparams.push_back(0);//do rotation

    if(bExactCOG)
    {
        ikparams.push_back(1);//converge to the COG no matter if it is in balance or not
        ikparams.push_back(cogtarg.x);
        ikparams.push_back(cogtarg.y);
        ikparams.push_back(cogtarg.z);
    }
    else
    {
        ikparams.push_back(0);//converge to the COG no matter if it is in balance or not
    }
        

    unsigned long starttime = timeGetTime();

    // for(vector<dReal>::iterator it = ikparams.begin(); it != ikparams.end(); it++)
    // {
    //    cout<<*it<<' ';
    // }
    // cout<<endl;

    boost::shared_ptr<std::vector<dReal> > pqResult(new std::vector<dReal> );
    //PrintMatrix(&ikparams[0], 1, ikparams.size(), "ikparams");

    // cout<<"Before IK Solver."<<endl;
    // int k;
    // k = getchar();

    if(_pIkSolver->Solve(IkParameterization(), q0, ikparams, false, pqResult) )
    {

        qResult = *pqResult.get();
        int timetaken = timeGetTime() - starttime;
        for(int i = 0; i < qResult.size(); i++)
        {
            sout << qResult[i] << " ";

        }
        if(bGetTime)
            sout << timetaken << " ";

        // RAVELOG_INFO("Solution Found! (%d ms)\n",timetaken);
        if(bExecute)
            robot->SetActiveDOFValues(qResult);
        
    }
    else
    {

        int timetaken = timeGetTime() - starttime;
        if(bReturnClosest)
        {
            qResult = *pqResult.get();
            for(int i = 0; i < qResult.size(); i++)
            {
                sout << qResult[i] << " ";
            }
        }

        if(bGetTime)
            sout << timetaken << " ";
        // RAVELOG_INFO("No IK Solution Found (%d ms)\n",timetaken);
    }

    // cout<<"After IK Solver."<<endl;
    // int l;
    // l = getchar();

    return true;

}


bool CBirrtProblem::CheckSupport(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    RAVELOG_DEBUG("Starting Check Support...\n");


    std::vector<dReal> polyx;
    std::vector<dReal> polyy;
    int nvert = 0;
    int numsupportlinks = 0;
    string cmd;
    std::vector<string> supportlinks;
    KinBodyPtr pheld;
    string tempstring;
    dReal temp;
    bool bdraw = false;
    Vector polyscale(1.0,1.0,1.0);
    Vector polytrans(0,0,0);
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if(stricmp(cmd.c_str(), "supportlinks") == 0 ){
            sinput >> numsupportlinks;
            for(int i =0; i < numsupportlinks; i++)
            {
                sinput >> tempstring;
                supportlinks.push_back(tempstring);
            }

        }
        else if( stricmp(cmd.c_str(), "heldobject") == 0 ) {
            sinput >> tempstring;
            pheld = GetEnv()->GetKinBody(tempstring.c_str());
            if(pheld == NULL)
            {
                RAVELOG_INFO("Error: could not find the specified held object\n");
                return false;
            }
        }
        else if(stricmp(cmd.c_str(), "polyscale") == 0 ){
            sinput >> polyscale.x;
            sinput >> polyscale.y;
            sinput >> polyscale.z;
        }
        else if(stricmp(cmd.c_str(), "polytrans") == 0 ){
            sinput >> polytrans.x;
            sinput >> polytrans.y;
            sinput >> polytrans.z;
        }
        else if(stricmp(cmd.c_str(), "draw") == 0 ){
            bdraw = true;
        }
        else break;


        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }
    
    GetSupportPolygon(supportlinks,polyx,polyy,polyscale,polytrans);
    int numPointsOut = polyx.size();

    Vector center;
    dReal fTotalMass = 0;
    std::vector<KinBody::LinkPtr>::const_iterator itlink;

    std::vector<KinBody::LinkPtr> vlinks = robot->GetLinks();
    if(pheld != NULL)
    {
        for(int i = 0; i < pheld->GetLinks().size(); i++)
            vlinks.push_back(pheld->GetLinks()[i]);
    }

    FORIT(itlink, vlinks) {
        //RAVELOG_INFO("Name %s comoffset: %f %f %f\n", (*itlink)->GetName().c_str(), (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
        if(bdraw)
        {
            GetEnv()->plot3(&(DoubleVectorToFloatVector((*itlink)->GetTransform() * (*itlink)->GetCOMOffset())[0]), 1, 0, (*itlink)->GetMass()/50, Vector(0,0,1),1 );
        }
        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        center /= fTotalMass;
    RAVELOG_INFO("\nmass: %f\ncog: %f %f %f\n",fTotalMass,center.x,center.y,center.z);


    dReal testx = center.x;
    dReal testy = center.y;
    dReal * vertx = &polyx[0];
    dReal * verty = &polyy[0];
    nvert = numPointsOut;

    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
             (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
            c = !c;
    }

    Vector cgline[2];
    cgline[0] = center;
    cgline[1] = center;
    cgline[1].z = 0;


    std::vector<RaveVector<float> > fcgline;
    for(int i =0; i < 2; i++)
        fcgline.push_back(RaveVector<float>(cgline[i].x,cgline[i].y,cgline[i].z));

    if(c)
    {
        //GetEnv()->plot3(center+1, 1+1, 0, 0.03, Vector(0,1,0),1 );
        if(bdraw)
        {
            GetEnv()->drawlinestrip(&(fcgline[0].x),2,sizeof(RaveVector<float>(0, 1, 0, 0)),5, RaveVector<float>(0, 1, 0, 0));
        }
        RAVELOG_INFO("Supported\n");
        sout << "1";
    }
    else
    {
        if(bdraw)
        {
            GetEnv()->drawlinestrip(&(fcgline[0].x),2,sizeof(RaveVector<float>(0, 1, 0, 0)),5, RaveVector<float>(1, 0, 0, 0));
        }
        //GetEnv()->plot3(center+1, 1+1, 0, 0.03, Vector(1,0,0),1 );
        RAVELOG_INFO("Not Supported\n");
        sout << "0";
    }

    return true;

}


void CBirrtProblem::PlannerWorker(PlannerBasePtr _pTCplanner, TrajectoryBasePtr ptraj, string filename)
{
    RAVELOG_INFO("Creating lock in new thread...\n");
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    RAVELOG_INFO("Lock created in new thread.\n");
    _plannerState = PS_Planning;
    OpenRAVE::PlannerStatus bSuccess = _pTCplanner->PlanPath(ptraj);
    //NOTE: _plannerState is a different class than OpenRAVE::PlannerStatus (includes a "Planning" state)
    if(bSuccess == PS_HasSolution)
    {
        WriteTraj(ptraj, filename);
        _plannerState = PS_PlanSucceeded;
    }
    else
    {
        _plannerState = PS_PlanFailed;
    }

    /* Reset joint limits */
    for (int j=0; j<_limadj_joints.size(); j++)
    {
        KinBody::JointPtr joint;
        vector<dReal> j_lower;
        vector<dReal> j_upper;
        joint = _limadj_joints[j];
        j_lower = _limadj_lowers[j];
        j_upper = _limadj_uppers[j];
        joint->SetLimits(j_lower, j_upper);
    }

    RAVELOG_INFO("Planner worker thread terminating.\n");
}

void CBirrtProblem::WriteTraj(TrajectoryBasePtr ptraj, string filename)
{
    //TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
    //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);

    // save the constrained trajectory

    //pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), true, false);
    //OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");

    //print out groups
    //    int numgroups = ptraj->GetConfigurationSpecification()._vgroups.size();
    //    RAVELOG_INFO("num groups in traj: %d\n",numgroups);
    //    for(int i = 0; i < numgroups; i++)
    //    {
    //        RAVELOG_INFO("name %d: %s\n",i,ptraj->GetConfigurationSpecification()._vgroups[i].name.c_str());
    //    }



    //this is the equivalent of GetFullTrajectoryFromActive
    ConfigurationSpecification activespec = ptraj->GetConfigurationSpecification();
    std::set<KinBodyPtr> sbodies;
    FOREACH(itgroup, activespec._vgroups) {
        stringstream ss(itgroup->name);
        string type, bodyname;
        ss >> type;
        ss >> bodyname;
        //RAVELOG_INFO("Bodyname: %s\n",bodyname.c_str());
        if(GetEnv()->GetKinBody(bodyname))
        {
            sbodies.insert(GetEnv()->GetKinBody(bodyname));
        }
    }

    ConfigurationSpecification spec;
    set <KinBodyPtr>::iterator si;
    for (si=sbodies.begin(); si!=sbodies.end(); si++)
    {
        std::vector<int> indices((*si)->GetDOF());
        for(int i = 0; i < (*si)->GetDOF(); ++i) {
            indices[i] = i;
        }
        ConfigurationSpecification tempspec = (*si)->GetConfigurationSpecificationIndices(indices, "linear");
        tempspec.AddDerivativeGroups(1,true); // add velocity + timestamp
        spec += tempspec;
    }

    OpenRAVE::planningutils::ConvertTrajectorySpecification(ptraj, spec);


    ofstream outfile(filename.c_str(),ios::out);
    outfile.precision(16);
    //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    ptraj->serialize(outfile);
    outfile.close();
    chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
}


int CBirrtProblem::GetPlannerState(ostream& sout, istream& sinput)
{
    PlannerState _currentState = _plannerState; //this should be atomic, so don't need a mutex

    switch (_currentState) {
    case PS_Idle:
        sout << "idle";
        break;
    case PS_Planning:
        sout << "planning";
        break;
    case PS_PlanSucceeded:
        sout << "plansucceeded";
        break;
    case PS_PlanFailed:
        sout << "planfailed";
        break;
    default:
        sout << "ERROR: UNKNOWN PLANNER STATE!";
        return 0;
    }

    return 1;
}

int CBirrtProblem::StopPlanner(ostream& sout, istream& sinput)
{

    PlannerState _currentState = _plannerState; //this should be atomic, so don't need a mutex

    switch (_currentState) {
    case PS_Idle:
        sout << "ERROR: PLANNER IS NOT CURRENTLY PLANNING!";
        return 0;
    case PS_Planning:
        //terminate the planner
        _plannerState = PS_PlanFailed; //this should be atomic, so don't need a mutex
        sout << "planner stopped";
        break;
    case PS_PlanSucceeded:
        sout << "planner already finished";
        break;
    case PS_PlanFailed:
        sout << "planner already finished";
        break;
    default:
        sout << "ERROR: UNKNOWN PLANNER STATE!";
        return 0;
    }

    return 1;
}

int CBirrtProblem::RunCBirrt(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock envlock(GetEnv()->GetMutex());

    boost::shared_ptr<CBirrtParameters> params;
    params.reset(new CBirrtParameters());
    std::vector<dReal> goals;
    std::vector<dReal> starts;

    bool bPlanInNewThread = false;
    bool bSmoothTrajOnly = false;
    int nvert = 0;
    int numsupportlinks = 0;
    std::vector<string> supportlinks;
    KinBodyPtr pheld;
    string tempstring;
    dReal temp_r;
    Vector polyscale(1.0,1.0,1.0);
    Vector polytrans(0,0,0);
    string cmd;
    bool bAllowLimAdj = false;
    KinBody::JointPtr joint;
    vector<dReal> j_lower;
    vector<dReal> j_upper;
    std::vector<string> support_manips; // from support
    std::vector<dReal> support_mus;     // from support
    std::vector<int> support_modes;     // from support
    Vector gravity(0.0, 0.0, -9.8);     // from gravity
    int balance_mode = 0;
    
    string filename = "cmovetraj.txt";
    string smoothtrajfilename;
    params->Tattachedik_0.resize(robot->GetManipulators().size());

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "jointgoals") == 0 ) {
            //note that this appends to goals, does not overwrite them
            int temp;
            sinput >> temp;
            int oldsize = goals.size();
            goals.resize(oldsize+temp);
            for(size_t i = oldsize; i < oldsize+temp; i++)
            {
                sinput >> goals[i];
            }
            
        }
        else if( stricmp(cmd.c_str(), "jointstarts") == 0 ) {
            //note that this appends to starts, does not overwrite them
            int temp;
            sinput >> temp;
            int oldsize = starts.size();
            starts.resize(oldsize+temp);
            for(size_t i = oldsize; i < oldsize+temp; i++)
            {
                sinput >> starts[i];
            }
            
        }
        else if( stricmp(cmd.c_str(), "ikguess") == 0 ) {
            int temp;
            sinput >> temp;
            params->vikguess.resize(temp);
            for(size_t i = 0; i < temp; i++)
            {
                sinput >> params->vikguess[i];
            }
            
        }
        else if( stricmp(cmd.c_str(), "Tattachedik_0") == 0) {
            int manipind;
            sinput >> manipind;
            TransformMatrix tmattachedik_0;
            sinput >> tmattachedik_0.m[0];
            sinput >> tmattachedik_0.m[4];
            sinput >> tmattachedik_0.m[8];
            sinput >> tmattachedik_0.m[1];
            sinput >> tmattachedik_0.m[5];
            sinput >> tmattachedik_0.m[9];
            sinput >> tmattachedik_0.m[2];
            sinput >> tmattachedik_0.m[6];
            sinput >> tmattachedik_0.m[10];
            sinput >> tmattachedik_0.trans.x;
            sinput >> tmattachedik_0.trans.y;
            sinput >> tmattachedik_0.trans.z;
            params->Tattachedik_0[manipind] = Transform(tmattachedik_0);
        }
        else if( stricmp(cmd.c_str(), "smoothingitrs") == 0) {
            sinput >> params->smoothingitrs;
        }
        else if( stricmp(cmd.c_str(), "filename") == 0) {
            sinput >> filename;
        }
        else if( stricmp(cmd.c_str(), "timelimit") == 0) {
            sinput >> params->timelimit;
        }
        else if( stricmp(cmd.c_str(), "TSRChain") == 0) {
            TaskSpaceRegionChain temp;
            temp.deserialize_from_matlab(robot,GetEnv(),sinput);
            params->vTSRChains.push_back(temp);
            if(temp.IsForStartSampling())
            {
                RAVELOG_INFO("Doing Start sampling\n");
                params->bsamplingstart = true;
            }
            if(temp.IsForGoalSampling())
            {
                RAVELOG_INFO("Doing Goal sampling\n");
                params->bsamplinggoal = true;
            }
        }
        else if(stricmp(cmd.c_str(), "supportlinks") == 0 ){
            sinput >> numsupportlinks;
            for(int i =0; i < numsupportlinks; i++)
            {
                sinput >> tempstring;
                supportlinks.push_back(tempstring);
            }
            balance_mode = 1;
        }
        else if(stricmp(cmd.c_str(), "polyscale") == 0 ){
            sinput >> polyscale.x;
            sinput >> polyscale.y;
            sinput >> polyscale.z;
        }
        else if(stricmp(cmd.c_str(), "polytrans") == 0 ){
            sinput >> polytrans.x;
            sinput >> polytrans.y;
            sinput >> polytrans.z;
        }
        else if (stricmp(cmd.c_str(), "support") == 0){
            // Support specifier for GIWC stability checking. May be provided more than once. Format is
            // "support {manip_index} {friction_coeff} {mode}" where mode is a bitmask indicating whether the support
            // link is active during the start, entire path, or end (expressed in binary). The most common modes are
            // 011 (link will be in contact during and after the motion) and 010 (link will be in contact during the
            // motion but removed immediately after the motion is finished, so the motion needs to end in such a way
            // that the robot is stable with or without that link). Start is not usually used and may be ommitted.
            string tempstr;
            sinput >> tempstr; support_manips.push_back(tempstr);
            sinput >> temp_r; support_mus.push_back(temp_r);
            sinput >> tempstr; support_modes.push_back(strtol(tempstr.c_str(), NULL, 2));
            balance_mode = 2;
        }
        else if (stricmp(cmd.c_str(), "gravity") == 0){
            sinput >> gravity.x;
            sinput >> gravity.y;
            sinput >> gravity.z;
        }
        else if( stricmp(cmd.c_str(), "heldobject") == 0 ) {
            sinput >> tempstring;
            pheld = GetEnv()->GetKinBody(tempstring.c_str());
            if(pheld == NULL)
            {
                RAVELOG_INFO("Error: could not find the specified held object\n");
                sout << 0;
                return -1;
            }
            robot->Grab(pheld);
            robot->CheckSelfCollision();
            params->bgrabbed = true;
        }
        else if( stricmp(cmd.c_str(), "psample") == 0)
        {
            sinput >> params->Psample;

        }
        else if( stricmp(cmd.c_str(), "bikfastsinglesolution") == 0)
        {
            sinput >> params->bikfastsinglesolution;
        }
        else if( stricmp(cmd.c_str(), "planinnewthread") == 0)
        {
            sinput >> bPlanInNewThread;
        }
        else if( stricmp(cmd.c_str(), "allowlimadj") == 0 )
        {
            sinput >> bAllowLimAdj;
        }
        else if( stricmp(cmd.c_str(), "smoothtrajonly") == 0)
        {
            bSmoothTrajOnly = true;
            sinput >> smoothtrajfilename;
            RAVELOG_INFO("smoothing only!\n");
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            sout << 0;
            return -1;
        }
    }

    if(supportlinks.size() != 0) {
        GetSupportPolygon(supportlinks,params->vsupportpolyx,params->vsupportpolyy,polyscale,polytrans);
    } else if (support_manips.size() != 0) {
        std::vector<string> curr_support_manips;
        std::vector<dReal> curr_support_mus;

        // First do start support
        curr_support_manips.clear();
        curr_support_mus.clear();
        for (int i = 0; i < support_manips.size(); i++) {
            if (support_modes[i] & 0b100) {
                curr_support_manips.push_back(support_manips[i]);
                curr_support_mus.push_back(support_mus[i]);
            }
        }
        if (curr_support_manips.size() > 0) {
            GetGIWC(curr_support_manips, curr_support_mus, params->vstartsupportcone);
        }

        // End support
        curr_support_manips.clear();
        curr_support_mus.clear();
        for (int i = 0; i < support_manips.size(); i++) {
            if (support_modes[i] & 0b001) {
                curr_support_manips.push_back(support_manips[i]);
                curr_support_mus.push_back(support_mus[i]);
            }
        }
        if (curr_support_manips.size() > 0) {
            GetGIWC(curr_support_manips, curr_support_mus, params->vendsupportcone);
        }

        // Path support
        curr_support_manips.clear();
        curr_support_mus.clear();
        for (int i = 0; i < support_manips.size(); i++) {
            if (support_modes[i] & 0b010) {
                curr_support_manips.push_back(support_manips[i]);
                curr_support_mus.push_back(support_mus[i]);
            }
        }
        if (curr_support_manips.size() > 0) {
            GetGIWC(curr_support_manips, curr_support_mus, params->vpathsupportcone);
        }

    }

    params->vgravity.push_back(gravity.x);
    params->vgravity.push_back(gravity.y);
    params->vgravity.push_back(gravity.z);


    if(starts.size() == 0 && !params->bsamplingstart)
    {
        RAVELOG_INFO("Setting default init config to current config\n");
        //default case: when not sampling starts and no starts specified, use current config as start
        params->vinitialconfig.resize(robot->GetActiveDOF());
        robot->GetActiveDOFValues(params->vinitialconfig);
    }
    else
    {
        //add any starts if they were specified
        for(int i = 0; i < starts.size(); i++)
            params->vinitialconfig.push_back(starts[i]);
    }


    //add any goals if they were specified
    for(int i = 0; i < goals.size(); i++)
        params->vgoalconfig.push_back(goals[i]);


    PlannerBasePtr _pTCplanner;


    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());


    _pTCplanner = RaveCreatePlanner(GetEnv(),"CBiRRT");
    if( _pTCplanner == NULL ) {
        RAVELOG_INFO("failed to create planner\n");
        sout << 0;
        return -1;
    }
    //ptraj->Clear();


    OpenRAVE::PlannerStatus bSuccess = PS_Failed;
    u32 startplan = timeGetTime();
    RAVELOG_DEBUG("starting planning\n");
    
    //for error reporting
    stringstream outputstream;
    stringstream command;
    command << "GetOutputMessage";



    /* Send over pointer to our planner state */
    params->pplannerstate = &_plannerState;

    if( !_pTCplanner->InitPlan(robot, params) ) {
        RAVELOG_INFO("InitPlan failed\n");
        _pTCplanner->SendCommand(outputstream,command);
        sout << 0 << " InitPlan failed\n" << outputstream.str();
        return -1;
    }


    /* Fix joint limits if current values are outside! */
    _limadj_joints.clear();
    _limadj_lowers.clear();
    _limadj_uppers.clear();
    if (bAllowLimAdj)
    {
        vector<dReal> start;
        vector<dReal> lower;
        vector<dReal> upper;
        vector<int> inds;
        vector<dReal> j_start;
        
        robot->GetActiveDOFValues(start);
        robot->GetActiveDOFLimits(lower, upper);
        inds = robot->GetActiveDOFIndices();
        
        /* Save all current joint values */
        for (int i=0; i<start.size(); i++) if (start[i] < lower[i] || start[i] > upper[i])
        {
            RAVELOG_INFO("Temporarily adjusting joint limit [%d].\n", i);
            joint = robot->GetJointFromDOFIndex(inds[i]);
            joint->GetLimits(j_lower, j_upper);
            _limadj_joints.push_back(joint);
            _limadj_lowers.push_back(j_lower);
            _limadj_uppers.push_back(j_upper);
        }
        for (int i=0; i<start.size(); i++) if (start[i] < lower[i] || start[i] > upper[i])
        {
            joint = robot->GetJointFromDOFIndex(inds[i]);
            joint->GetValues(j_start);
            joint->GetLimits(j_lower, j_upper);
            for (int j=0; j<j_start.size(); j++)
            {
                if (j_start[j] < j_lower[j]) j_lower[j] = j_start[j];
                if (j_start[j] > j_upper[j]) j_upper[j] = j_start[j];
            }
            joint->SetLimits(j_lower, j_upper);
        }
    }

    _plannerState = PS_Idle;


    if(bPlanInNewThread)
    {
        RAVELOG_INFO("Launching planner in new thread\n");
        _plannerThread.reset(new boost::thread(boost::bind(&CBirrtProblem::PlannerWorker, this, _1, _2, _3),_pTCplanner,ptraj, filename));
        _pTCplanner->SendCommand(outputstream,command);
        _plannerThread->detach();
        sout << 1 << " " << outputstream.str();
        return 1;
    }
    else
    {
        _plannerState = PS_Planning;
        if(!bSmoothTrajOnly)
            bSuccess = _pTCplanner->PlanPath(ptraj);
        else
        {
            TrajectoryBasePtr ptraj_in = RaveCreateTrajectory(GetEnv(),"");
            ptraj_in->Init(robot->GetConfigurationSpecification());

            RAVELOG_DEBUG("CBiRRTProblem: reading trajectory: %s\n", smoothtrajfilename.c_str());
            ifstream infile(smoothtrajfilename.c_str(),ios::in);
            //if( !ptraj_in->Read(infile, robot) ) {
            if( !ptraj_in->deserialize(infile) ) {
                RAVELOG_FATAL("CBiRRTProblem: failed to read trajectory %s\n", smoothtrajfilename.c_str());
                infile.close();
                /* Reset joint limits */
                for (int j=0; j<_limadj_joints.size(); j++)
                {
                    joint = _limadj_joints[j];
                    j_lower = _limadj_lowers[j];
                    j_upper = _limadj_uppers[j];
                    joint->SetLimits(j_lower, j_upper);
                }
                _pTCplanner->SendCommand(outputstream,command);
                sout << 0 << " CBiRRTProblem: failed to read trajectory " << smoothtrajfilename << endl << outputstream.str();
                return -1;
            }


            bool bError = ((CBirrtPlanner *)_pTCplanner.get())->SetVecPathFromTraj(ptraj_in);
            if(!bError)
                bSuccess = PS_HasSolution;
            bool bTerminated = false;
            double starttime = timeGetThreadTime();
            ((CBirrtPlanner *)_pTCplanner.get())->_OptimizePath(bTerminated, starttime);
            ((CBirrtPlanner *)_pTCplanner.get())->_CreateTraj(ptraj);


        }
        if(bSuccess == PS_HasSolution)
            _plannerState = PS_PlanSucceeded;
        else
            _plannerState = PS_PlanFailed;
    }




    _plannerState = PS_Idle;
    
    /* Reset joint limits */
    for (int j=0; j<_limadj_joints.size(); j++)
    {
        joint = _limadj_joints[j];
        j_lower = _limadj_lowers[j];
        j_upper = _limadj_uppers[j];
        joint->SetLimits(j_lower, j_upper);
    }

    if(bSuccess != PS_HasSolution)
    {
        _pTCplanner->SendCommand(outputstream,command);
        sout << 0 << " " << outputstream.str();
        return -1;
    }

    WriteTraj(ptraj, filename);
    _pTCplanner->SendCommand(outputstream,command);
    sout << 1 << " " << outputstream.str();
    return 1;
}



string getfilename_withseparator(istream& sinput, char separator)
{
    string filename;
    if( !getline(sinput, filename, separator) ) {
        // just input directly
        RAVELOG_ERRORA("filename not terminated with ';'\n");
        sinput >> filename;
    }

    // trim leading spaces
    size_t startpos = filename.find_first_not_of(" \t");
    size_t endpos = filename.find_last_not_of(" \t");

    // if all spaces or empty return an empty string
    if(( string::npos == startpos ) || ( string::npos == endpos))
        return "";

    filename = filename.substr( startpos, endpos-startpos+1 );
    return filename;
}

bool CBirrtProblem::GetJointAxis(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    RAVELOG_DEBUG("Starting GetJointAxis...\n");

    KinBodyPtr pobject;
    int jointind = -1;

    dReal temp;
    string cmd, tempstring;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "name") == 0 ) {
            sinput >> tempstring;
            pobject = GetEnv()->GetKinBody(tempstring.c_str());
            if(pobject == NULL)
            {
                RAVELOG_INFO("Error: could not find the specified object to get axis\n");
                return false;
            }
        }
        else if( stricmp(cmd.c_str(), "jointind") == 0 ) {
            sinput >> jointind;
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }


    std::vector<KinBody::JointPtr> _vecjoints = pobject->GetJoints();

    if(jointind >= _vecjoints.size() || jointind < 0)
    {
        RAVELOG_INFO("Joint index out of bounds or not specified\n");
        return false;
    }

    Vector axis = _vecjoints[jointind]->GetAxis(0);

    sout << axis.x << " " << axis.y << " " << axis.z;

    return true;
}

bool CBirrtProblem::GetJointTransform(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    RAVELOG_DEBUG("Starting GetJointTransform...\n");

    KinBodyPtr pobject;
    int jointind = -1;

    dReal temp;
    string cmd, tempstring;

    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "name") == 0 ) {
            sinput >> tempstring;
            pobject = GetEnv()->GetKinBody(tempstring.c_str());
            if(pobject == NULL)
            {
                RAVELOG_INFO("Error: could not find the specified object to get transform\n");
                return false;
            }
        }
        else if( stricmp(cmd.c_str(), "jointind") == 0 ) {
            sinput >> jointind;
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }


    std::vector<KinBody::JointPtr> _vecjoints = pobject->GetJoints();

    if(jointind >= _vecjoints.size() || jointind < 0)
    {
        RAVELOG_INFO("Joint index out of bounds or not specified\n");
        return false;
    }

    TransformMatrix jointtm = _vecjoints[jointind]->GetFirstAttached()->GetTransform();
    jointtm.trans = _vecjoints[jointind]->GetAnchor();
    sout << jointtm.m[0] << " ";
    sout << jointtm.m[4] << " ";
    sout << jointtm.m[8] << " ";
    sout << jointtm.m[1] << " ";
    sout << jointtm.m[5] << " ";
    sout << jointtm.m[9] << " ";
    sout << jointtm.m[2] << " ";
    sout << jointtm.m[6] << " ";
    sout << jointtm.m[10] << " ";
    sout << jointtm.trans.x << " ";
    sout << jointtm.trans.y << " ";
    sout << jointtm.trans.z << " ";

    return true;
}


bool CBirrtProblem::GrabBody(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    RAVELOG_DEBUG("Starting GrabBody...\n");

    KinBodyPtr ptarget;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "name") == 0 ) {
            string name;
            sinput >> name;
            ptarget = GetEnv()->GetKinBody(name.c_str());
        }
        else break;

        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if(ptarget == NULL) {
        RAVELOG_INFO("ERROR Manipulation::GrabBody - Invalid body name.\n");
        return false;
    }
    
    robot->Grab(ptarget);
    return true;
}

int CBirrtProblem::convexHull2D(coordT* pointsIn, int numPointsIn, coordT** pointsOut, int* numPointsOut) {

    char flags[250];
    int exitcode;
    facetT *facet, *newFacet;
    int curlong, totlong;
    vertexT *vertex, *vertexA, *vertexB;
    int j;


    sprintf (flags, "qhull QJ Pp s Tc ");
    //FILE* junk = fopen("qhullout.txt","w");

    exitcode = qh_new_qhull(2, numPointsIn, pointsIn, false,
                            flags, NULL, stderr);
    //fclose(junk);
    *numPointsOut = qh num_vertices;
    *pointsOut = (coordT *)malloc(sizeof(coordT)*(*numPointsOut)*2);

    FORALLfacets {
        facet->seen = 0;
    }

    FORALLvertices {
        vertex->seen = 0;
    }

    facet=qh facet_list;
    j=0;

    while(1) {
        if (facet==NULL) {
            // empty hull
            break;
        }
        vertexA = (vertexT*)facet->vertices->e[0].p;
        vertexB = (vertexT*)facet->vertices->e[1].p;
        if (vertexA->seen==0) {
            vertexA->seen = 1;
            (*pointsOut)[j++] = vertexA->point[0];
            (*pointsOut)[j++] = vertexA->point[1];
        }
        if (vertexB->seen==0) {
            vertexB->seen = 1;
            (*pointsOut)[j++] = vertexB->point[0];
            (*pointsOut)[j++] = vertexB->point[1];
        }


        //qh_printfacet(stderr, facet);
        facet->seen = 1;
        newFacet = (facetT*)facet->neighbors->e[0].p;
        if (newFacet->seen==1) newFacet = (facetT*)facet->neighbors->e[1].p;
        if (newFacet->seen==1) { break; }
        facet = newFacet;
    }

    qh_freeqhull(!qh_ALL);
    qh_memfreeshort (&curlong, &totlong);

    return exitcode;
}

int CBirrtProblem::convexHull6D(coordT* pointsIn, int numPointsIn, std::vector< std::vector<double> >& facet_coefficients) {

    char flags[250];
    int exitcode;
    facetT *facet, *newFacet;
    int curlong, totlong;
    facet_coefficients.clear();

    float min_value = 0.000001;

    sprintf (flags, "qhull QJ Pp s Tc ");

    // cout<<"input points."<<endl;
    // for(int i = 0; i < numPointsIn; i++)
    // {
    //     cout<<pointsIn[i*6+0]<<" "<<pointsIn[i*6+1]<<" "<<pointsIn[i*6+2]<<" "<<pointsIn[i*6+3]<<" "<<pointsIn[i*6+4]<<" "<<pointsIn[i*6+5]<<endl;
    // }

    exitcode= qh_new_qhull (6, numPointsIn, pointsIn, false, flags, NULL, stderr);

    FORALLfacets {
        facet->seen = 0;
    }

    facet = qh facet_list;
    int facet_numbers = qh num_facets;

    set<string> facet_hash_set;
    stringstream coeff_string;
    int rounded_coeff;

    // cout<<"facet parameters"<<endl;

    for(int i = 0; i < facet_numbers; i++)
    {
        // get the facet equations.
        coordT* normal = facet->normal;
        coordT offset = facet->offset;
        
        string hash = "";

        if(fabs(offset) < 0.0001)
        {
            std::vector<double> coeff(7);
            if(fabs(offset) < min_value)
            {
                coeff[0] = 0;
            }
            else
            {
                coeff[0] = offset;
            }
            rounded_coeff = round(coeff[0]*1000);
            if(rounded_coeff == 0) rounded_coeff = 0; // eliminate negative 0
            coeff_string << rounded_coeff;
            hash = hash + coeff_string.str();
            coeff_string.str(std::string());
            coeff_string.clear();

            for(int i = 0; i < 6; i++)
            {
                if(fabs(normal[i]) < min_value)
                {
                    coeff[i+1] = 0;
                }
                else
                {
                    coeff[i+1] = normal[i];
                }
                rounded_coeff = round(coeff[i+1]*1000);
                if(rounded_coeff == 0) rounded_coeff = 0; // eliminate negative 0
                coeff_string << rounded_coeff;
                hash = hash + "," + coeff_string.str();
                coeff_string.str(std::string());
                coeff_string.clear();
            }

            if(facet_hash_set.find(hash) == facet_hash_set.end())
            {
                facet_coefficients.push_back(coeff);
                facet_hash_set.insert(hash);
                // cout<<hash<<endl;
                // cout<<coeff[0]<<" "<<coeff[1]<<" "<<coeff[2]<<" "<<coeff[3]<<" "<<coeff[4]<<" "<<coeff[5]<<" "<<coeff[6]<<endl;
            }

        }
        if(i != facet_numbers-1)
        {
            newFacet = (facetT*)facet->next;
            facet = newFacet;
        }

    }

    qh_freeqhull(!qh_ALL);
    qh_memfreeshort (&curlong, &totlong);

    return exitcode;
}



void CBirrtProblem::compute_convex_hull(void)
{  

    int dim =2;                   /* dimension of points */
    int numpoints = 4;            /* number of points */
    coordT points[8] = {0, 0,  1, 0,  1, 1,  0, 1};           /* array of coordinates for each point */
    boolT ismalloc = false;           /* True if qhull should free points in qh_freeqhull() or reallocation */
    char flags[]= "qhull Tv"; /* option flags for qhull, see qh_opt.htm */
    FILE *outfile= stdout;    /* output from qh_produce_output()
                                 use NULL to skip qh_produce_output() */
    FILE *errfile= stderr;    /* error messages from qhull code */
    int exitcode;             /* 0 if no error from qhull */
    facetT *facet;            /* set by FORALLfacets */
    int curlong, totlong;     /* memory remaining after qh_memfreeshort */

    /* initialize dim, numpoints, points[], ismalloc here */
    exitcode= qh_new_qhull (dim, numpoints, points, ismalloc,
                            flags, outfile, errfile);
    if (!exitcode) { /* if no error */
        /* 'qh facet_list' contains the convex hull */
        FORALLfacets {
            /* ... your code ... */
        }
    }
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong)
        fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
                 totlong, curlong);

    RAVELOG_INFO("qhull done\n");
}

void draw_cone(EnvironmentBasePtr env, const NEWMAT::Matrix& mat, const Transform& manip_tf, int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        Vector force(mat(r+1, 1), mat(r+1, 2), mat(r+1, 3), 1);
        force *= 0.2; // Scale down the arrow
        graphptrs.push_back(env->drawarrow(manip_tf.trans, manip_tf * force, 0.001, RaveVector<float>(1,0.5,0.5,1)));

        if (cols > 3) {
            Vector torque(mat(r+1, 4), mat(r+1, 5), mat(r+1, 6), 1);
            torque *= 0.2; // Scale down the arrow
            graphptrs.push_back(env->drawarrow(manip_tf.trans, manip_tf * torque, 0.002, RaveVector<float>(0.5,1,0.5,1)));
        }
    }
}

void draw_tf(EnvironmentBasePtr env, const Transform& tf, const Transform& origin_tf) {
    Transform total_tf = origin_tf * tf;
    Vector origin = total_tf * Vector(0, 0, 0, 1);
    graphptrs.push_back(env->drawarrow(origin, total_tf * Vector(0.1, 0, 0, 1), 0.001, Vector(1, 0, 0, 1)));
    graphptrs.push_back(env->drawarrow(origin, total_tf * Vector(0, 0.1, 0, 1), 0.001, Vector(0, 1, 0, 1)));
    graphptrs.push_back(env->drawarrow(origin, total_tf * Vector(0, 0, 0.1, 1), 0.001, Vector(0, 0, 1, 1)));
}

/// Returns the support points relative to the world frame
void CBirrtProblem::GetSupportPointsForLink(RobotBase::LinkPtr p_link, Vector tool_dir, Transform result_tf, std::vector<Vector>& contacts) {
    Transform tf = result_tf.inverse() * p_link->GetTransform();

    AABB aabb = p_link->ComputeLocalAABB();

    // If any extent is 0, the link has no volume and is assumed to be a virtual link
    if (aabb.extents.x <= 0 || aabb.extents.y <= 0 || aabb.extents.z <= 0) {
        return;
    }

    if(strcmp(p_link->GetName().c_str(), "l_foot") != 0 &&
       strcmp(p_link->GetName().c_str(), "r_foot") != 0 &&
       strcmp(p_link->GetName().c_str(), "l_palm") != 0 &&
       strcmp(p_link->GetName().c_str(), "r_palm") != 0)
    {
        return;
    }

    // Iterates over the 8 combinations of (+ or -) for each of the 3 dimensions
    for (int neg_mask = 0; neg_mask < 8; neg_mask++) {
        Vector contact;
        bool is_invalid = false;

        // Iterate over x, y, and z (compiler probably unrolls this loop)
        for (int axis = 0; axis < 3; axis++) {
            bool neg = !!(neg_mask&(1<<axis));
            // A point will be "invalid" if it is opposite the local tool direction
            is_invalid = is_invalid || (neg && (tool_dir[axis] > 0.85)) || (!neg && (tool_dir[axis] < -0.85));
            if (is_invalid) break;

            contact[axis] = aabb.pos[axis] + (neg ? -1 : 1)*aabb.extents[axis];
        }

        if (!is_invalid) {
            Vector contact_t = tf * contact;
            contacts.push_back(contact_t);

//            return; //! TEMP
        }
    }

}

/// Returns the support points of the given manipulator in the WORLD frame
std::vector<Vector> CBirrtProblem::GetSupportPoints(RobotBase::ManipulatorPtr p_manip) {
    // Get all rigidly attached links -- in my case, the end effector link is a virtual
    // link with 0 volume. The actual physical ee link is rigidly attached to it.
    std::vector<RobotBase::LinkPtr> attached_links;
    p_manip->GetEndEffector()->GetRigidlyAttachedLinks(attached_links);

    // Other manipulator info
    Transform world_to_manip = p_manip->GetTransform().inverse();
    Vector tool_dir = p_manip->GetLocalToolDirection();

    std::vector<Vector> contacts;
    for (int i = 0; i < attached_links.size(); i++) {
        const char* link_name = attached_links[i]->GetName().c_str();

        // Transforms the tool_dir into the link frame
        GetSupportPointsForLink(attached_links[i], world_to_manip * attached_links[i]->GetTransform() * tool_dir, p_manip->GetTransform(), contacts);

    }
    return contacts;
}

const int CONE_DISCRETIZATION_RESOLUTION = 4;
void CBirrtProblem::GetFrictionCone(Vector& center, Vector& direction, dReal mu, NEWMAT::Matrix* mat, int offset_r, int offset_c, Transform temp_tf) {
    // This sets `a` (for axis) to the index of `direction` that is nonzero, sets `c` (cos) to the first index that
    // is zero, and `s` (sin) to the second that is zero. Formulas derived from a truth table.
    // NOTE: 1-based indexing
    int a = direction[0] ? 1 : direction[1] ? 2 : 3;
    int c = 1 + (a == 1); // 1 if `a` is not 1, 2 otherwise.
    int s = 3 - (a == 3); // 3 if `a` is not 3, 2 otherwise

    dReal step = M_PI * 2.0 / CONE_DISCRETIZATION_RESOLUTION;
    dReal angle = 0;
    // NOTE 1-based indexing
    for (int i = 1; i <= CONE_DISCRETIZATION_RESOLUTION; i++) {
        // a-column will be -1 or 1. The -1 multiplication is because friction force occurs in the opposite direction
        // (*mat)(offset_r + i, offset_c + a) = center[a-1] + direction[a-1] * -1;
        // (*mat)(offset_r + i, offset_c + s) = center[s-1] + round(mu * sin(angle) * 10000) * 0.0001;
        // (*mat)(offset_r + i, offset_c + c) = center[c-1] + round(mu * cos(angle) * 10000) * 0.0001;
        (*mat)(offset_r + i, offset_c + a) = direction[a-1] * -1;
        (*mat)(offset_r + i, offset_c + s) = round(mu * sin(angle) * 10000) * 0.0001;
        (*mat)(offset_r + i, offset_c + c) = round(mu * cos(angle) * 10000) * 0.0001;
        angle += step;
    }

}

void CBirrtProblem::GetASurf(RobotBase::ManipulatorPtr p_manip, Transform cone_to_manip, NEWMAT::Matrix *mat, int offset_r) {
    Transform manip_to_world = p_manip->GetTransform(); // For testing
    TransformMatrix tf_matrix = TransformMatrix(cone_to_manip); //TransformMatrix(cone_to_manip);

    // First 3 columns are just the tf_matrix
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            (*mat)(offset_r + r + 1, c + 1) =  tf_matrix.m[ r*4 + c ];
        }
    }

    // (Notes originally written for Python)
    // To calculate v, we take the origin of the friction cone in the world frame with cone_tf.trans,
    // then transform it to the surface frame by multipling by T_W_s = T_s_W.I. This gives the displacement from
    // surface frame to contact frame. To get the displacement from contact frame to surface frame, it is multiplied
    // by -1 to reverse direction b/c disp(a, b) = -disp(b, a).
    Vector v = (cone_to_manip.trans);
    v *= -1;
    // NOTE The above math messes up the 4th element of v, which must be 1 for affine transforms
    v[3] = 1;

    // Last 3 columns are the cross-product-equivalent matrix for r
    (*mat)(offset_r + 1, 4) =  0.0;
    (*mat)(offset_r + 1, 5) = -v.z;
    (*mat)(offset_r + 1, 6) =  v.y;
    (*mat)(offset_r + 2, 4) =  v.z;
    (*mat)(offset_r + 2, 5) =  0.0;
    (*mat)(offset_r + 2, 6) = -v.x;
    (*mat)(offset_r + 3, 4) = -v.y;
    (*mat)(offset_r + 3, 5) =  v.x;
    (*mat)(offset_r + 3, 6) =  0.0;
}

void CBirrtProblem::GetAStance(Transform tf, NEWMAT::Matrix* mat, int offset_r) {
    // Note: This AStance computes the transpose of the AStance from the GIWC paper,
    // because of the way the cone representation is done here
    // TransformMatrix m = TransformMatrix(tf.inverse());
    TransformMatrix m = TransformMatrix(tf);

    // Create -R matrix
    NEWMAT::Matrix negR_T(3, 3);
    negR_T << -m.m[0] << -m.m[4] << -m.m[8]
           << -m.m[1] << -m.m[5] << -m.m[9]
           << -m.m[2] << -m.m[6] << -m.m[10];

    // Create the transpose of the cross-product-equivalent matrix of the transform's translation component
    NEWMAT::Matrix crossP_T(3, 3);
    crossP_T <<  0.0        <<  tf.trans.z << -tf.trans.y
             << -tf.trans.z <<  0.0        <<  tf.trans.x
             <<  tf.trans.y << -tf.trans.x <<  0.0       ;

    // Create TRANSPOSE OF matrix [    -R        0 ]
    //                            [ [p]x * -R   -R ]
    (*mat).SubMatrix(offset_r + 1, offset_r + 3, 1, 3) = negR_T;
    // Computes transpose of multiplication by using (negR * crossP)_T = negR_T * crossP_T
    (*mat).SubMatrix(offset_r + 1, offset_r + 3, 4, 6) = negR_T * crossP_T;
    (*mat).SubMatrix(offset_r + 4, offset_r + 6, 1, 3) = 0.0;
    (*mat).SubMatrix(offset_r + 4, offset_r + 6, 4, 6) = negR_T;

    // for(int i = 1; i <= mat->Nrows(); i++)
    // {
    //     for(int j = 1; j <= mat->Ncols(); j++)
    //     {
    //         (*mat)(i,j) = 0.001 * round((*mat)(i,j) * 1000.0);
    //     }
    // }
    negR_T.ReleaseAndDelete();
    crossP_T.ReleaseAndDelete();
}

NEWMAT::ReturnMatrix CBirrtProblem::GetSurfaceCone(string& manipname, dReal mu) {

    if(_computed_contact_surface_cones.count(manipname) != 0){
        return _computed_contact_surface_cones.find(manipname)->second;
    }
    else{
        RobotBase::ManipulatorPtr p_manip = robot->GetManipulator(manipname);
        Vector manip_dir = p_manip->GetLocalToolDirection();

        std::vector<Vector> support_points = GetSupportPoints(p_manip);
        int num_points = support_points.size();

        // Calculate combined friction cone matrix
        int rows = CONE_DISCRETIZATION_RESOLUTION*num_points;
        int cols = 3*num_points;

        NEWMAT::Matrix f_cones_diagonal(rows, cols);
        f_cones_diagonal = 0.0; // All non-filled spots should be 0

        // Fill the diagonals
        for (int i = 0; i < num_points; i++) {
            GetFrictionCone(support_points[i], manip_dir, mu, &f_cones_diagonal, i*CONE_DISCRETIZATION_RESOLUTION, i*3, p_manip->GetTransform());
        }

        // Calculate A_surf matrix
        NEWMAT::Matrix a_surf_stacked(cols, 6); // TODO: Rename `cols` because it's the rows here

        for (int i = 0; i < num_points; i++) {
            // Cone transform has no rotation relative to the manipulator's transform and is translated by
            // the vector contained in support_points
            Transform cone_tf;
            cone_tf.trans = support_points[i];
            GetASurf(p_manip, cone_tf, &a_surf_stacked, 3*i);
        }

        NEWMAT::Matrix mat = f_cones_diagonal * a_surf_stacked; // Dot product

        // omit redundant rows, useless, no redundant rows
        dd_ErrorType err;
        dd_MatrixPtr contact_span_cdd = dd_CreateMatrix(mat.Nrows(), mat.Ncols()+1);
        for (int r = 0; r < mat.Nrows(); r++) {
        // First element of each row indicates whether it's a point or ray. These are all rays, indicated by 0.
            dd_set_si(contact_span_cdd->matrix[r][0], 0.0);
            for (int c = 0; c < mat.Ncols(); c++) {
                dd_set_si(contact_span_cdd->matrix[r][c+1], mat(r+1, c+1));
            }
        }

        // dd_rowset redundant_rows = dd_RedundantRows(contact_span_cdd,&err);

        // int redundant_rows_num = sizeof(redundant_rows) / sizeof(unsigned long);

        // NEWMAT::Matrix reduced_mat(0,6);

        // for(int i = 0; i < mat.Nrows(); i++)
        // {
        //     bool redundant = false;
        //     for(int j = 0; j < redundant_rows_num; j++)
        //     {
        //         if(i == redundant_rows[j])
        //         {
        //             redundant = true;
        //             break;
        //         }
                
        //     }

        //     if(!redundant)
        //     {
        //         reduced_mat &= mat.Row(i+1);
        //     }
        // }

        // _computed_contact_surface_cones.insert(std::pair<string,NEWMAT::Matrix>(manipname,reduced_mat));
        _computed_contact_surface_cones.insert(std::pair<string,NEWMAT::Matrix>(manipname,mat));

        dd_FreeMatrix(contact_span_cdd);
        f_cones_diagonal.ReleaseAndDelete();
        a_surf_stacked.ReleaseAndDelete();
        mat.Release();
        // reduced_mat.Release();
        // return reduced_mat;
        return mat;
    }
}

NEWMAT::ReturnMatrix CBirrtProblem::GetGIWCSpanForm(std::vector<std::string>& manip_ids, std::vector<dReal>& friction_coeffs) {
    int num_manips = manip_ids.size();
    std::vector<NEWMAT::Matrix> matrices;
    int total_rows = 0;

    for (int i = 0; i < num_manips; i++) {
        matrices.push_back(GetSurfaceCone(manip_ids[i], friction_coeffs[i]));
        total_rows += matrices.back().Nrows();
    }

    // RAVELOG_INFO("num_manips: %d\n",num_manips);

    // Calculate combined surface cone matrix
    NEWMAT::Matrix s_cones_diagonal(total_rows, 6*num_manips);
    s_cones_diagonal = 0.0;

    int current_row_offset = 0;
    for (int i = 0; i < num_manips; i++) {
        s_cones_diagonal.SubMatrix(current_row_offset+1, current_row_offset+matrices[i].Nrows(), (6*i)+1, (6*i)+6) = matrices[i];
        current_row_offset += matrices[i].Nrows();
    }

    // Calculate A_stance matrix
    NEWMAT::Matrix a_stance_stacked(6 * num_manips, 6);

    for (int i = 0; i < num_manips; i++) {
        GetAStance(robot->GetManipulator(manip_ids[i])->GetTransform(), &a_stance_stacked, 6*i);
    }

    NEWMAT::Matrix mat = s_cones_diagonal * a_stance_stacked; // Dot product

    // RAVELOG_INFO("s_cones_diagonal Rows: %d, Col: %d\n",s_cones_diagonal.Nrows(),s_cones_diagonal.Ncols());
    // cout<<s_cones_diagonal<<endl;
    // RAVELOG_INFO("a_stance_stacked Rows: %d, Col: %d\n",a_stance_stacked.Nrows(),a_stance_stacked.Ncols());
    // cout<<a_stance_stacked<<endl;
    // RAVELOG_INFO("mat Rows: %d, Col: %d\n",mat.Nrows(),mat.Ncols());

    s_cones_diagonal.ReleaseAndDelete();
    a_stance_stacked.ReleaseAndDelete();
    for(unsigned int i = 0; i < matrices.size(); i++)
    {
        matrices[i].ReleaseAndDelete();
    }
    mat.Release();
    return mat;
}

void CBirrtProblem::GetGIWC(std::vector<std::string>& manip_ids, std::vector<dReal>& mus, std::vector<dReal>& ikparams) {

    // unsigned long start = timeGetTime();
    
    // cout<<"Beginning of GetGIWC."<<endl;
    // int a;
    // a = getchar();
    
    dd_set_global_constants();

    // cout<<"dd_set_global_constraint."<<endl;
    // int b;
    // b = getchar();

    dd_ErrorType err;

    NEWMAT::Matrix giwc_span = GetGIWCSpanForm(manip_ids, mus);
    // RAVELOG_INFO("giwc_span_cdd Rows: %d, Col: %d\n",giwc_span.Nrows(),giwc_span.Ncols());


    // int al;
    // std::cin>>al;

    // unsigned long after_dd_getspanform = timeGetTime();
    // RAVELOG_INFO("after_dd_getspanform: %d\n",(after_dd_getspanform-start));

    // graphptrs.clear();
    // draw_cone(GetEnv(), giwc_span, Transform(), giwc_span.Nrows(), giwc_span.Ncols());

    // dd_MatrixPtr giwc_span_cdd = dd_CreateMatrix(giwc_span.Nrows(), giwc_span.Ncols()+1);
    // giwc_span_cdd->representation = dd_Generator;

    // unsigned long after_dd_creatematrix = timeGetTime();
    // RAVELOG_INFO("after_dd_creatematrix: %d\n",(after_dd_creatematrix-after_dd_getspanform));

    // cout<<"giwc_span_cdd."<<endl;
    // int c;
    // c = getchar();

    std::vector<coordT> pointsIn((giwc_span.Nrows()+1)*6);
    pointsIn[0] = 0;
    pointsIn[1] = 0;
    pointsIn[2] = 0;
    pointsIn[3] = 0;
    pointsIn[4] = 0;
    pointsIn[5] = 0;

    // TODO: Is there a better way than doing this?
    for (int r = 0; r < giwc_span.Nrows(); r++) {
        // First element of each row indicates whether it's a point or ray. These are all rays, indicated by 0.
        // dd_set_si(giwc_span_cdd->matrix[r][0], 0.000001);
        for (int c = 0; c < giwc_span.Ncols(); c++) {
            // It's legal to multiply an entire row by the same value (here 1e4)
            // This rounds everything down to a fixed precision int
            // dd_set_si(giwc_span_cdd->matrix[r][c+1], (long) (giwc_span(r+1, c+1) * 1e4));
            // dd_set_si(giwc_span_cdd->matrix[r][c+1], round(giwc_span(r+1, c+1) * 10000));
            pointsIn[(r+1)*6 + c] = 0.0001 * round(giwc_span(r+1, c+1) * 10000);
        }
    }

    // unsigned long after_dd_set_si = timeGetTime();
    // RAVELOG_INFO("after_dd_set_si: %d\n",(after_dd_set_si-after_dd_getspanform));

    
    int numPointsIn = giwc_span.Nrows()+1;
    std::vector< std::vector<double> > facet_coefficients;
    convexHull6D(&pointsIn[0], numPointsIn, facet_coefficients);

    // cout<<"dd_set_si."<<endl;
    // int d;
    // d = getchar();

    // unsigned long after_qhull = timeGetTime();
    // RAVELOG_INFO("after_qhull: %d\n",(after_qhull-after_dd_set_si));

    // dd_PolyhedraPtr poly = dd_DDMatrix2Poly2(giwc_span_cdd, dd_MaxCutoff, &err);
    // if (err != dd_NoError) {
    //     RAVELOG_INFO("CDD Error: ");
    //     dd_WriteErrorMessages(stdout, err);
    //     throw OPENRAVE_EXCEPTION_FORMAT("CDD Error: %d", err, ORE_InvalidState);
    // }

    // cout<<"poly."<<endl;
    // int e;
    // e = getchar();

    // unsigned long after_dd_matrix2poly2 = timeGetTime();
    // RAVELOG_INFO("after_dd_matrix2poly2: %d\n",(after_dd_matrix2poly2-after_dd_set_si));

    // dd_MatrixPtr giwc_face_cdd = dd_CopyInequalities(poly);

    // RAVELOG_INFO("Ustance Size:(%d,%d)",giwc_face_cdd->rowsize,giwc_face_cdd->colsize);

    // cout<<"giwc_face_cdd."<<endl;
    // int f;
    // f = getchar();

    // Add to ikparams
    prev_giwc.clear();
    // ikparams.push_back(giwc_face_cdd->rowsize);
    // prev_giwc.push_back(giwc_face_cdd->rowsize);

    // for (int row = 0; row < giwc_face_cdd->rowsize; row++) {
    //     dd_Arow point;
    //     // if(dd_Redundant(giwc_face_cdd,row,point,&err) == 0)
    //     if(true)
    //     {
    //         // Note this skips element 0 of each row, which should always be 0
    //         for (int col = 0; col < giwc_face_cdd->colsize; col++) {
    //             cout<<dd_get_d(giwc_face_cdd->matrix[row][col])<<" ";
    //         }
    //         cout<<endl;
    //     }
    // }

    // for (int row = 0; row < giwc_face_cdd->rowsize; row++) {
    //     // Note this skips element 0 of each row, which should always be 0
    //     for (int col = 1; col < giwc_face_cdd->colsize; col++) {
    //         ikparams.push_back(dd_get_d(giwc_face_cdd->matrix[row][col]));
    //         prev_giwc.push_back(ikparams[ikparams.size()-1]);
    //     }
    // }

    ikparams.push_back(facet_coefficients.size());
    prev_giwc.push_back(facet_coefficients.size());
    for (int row = 0; row < facet_coefficients.size(); row++) {
        // Note this skips element 0 of each row, which should always be 0
        for (int col = 1; col < 7; col++) {
            ikparams.push_back(-facet_coefficients[row][col]);
            prev_giwc.push_back(ikparams[ikparams.size()-1]);
        }
    }

    giwc_span.ReleaseAndDelete();
    // dd_FreeMatrix(giwc_face_cdd);
    // cout<<"Free giwc_face_cdd."<<endl;
    // int g;
    // g = getchar();
    
    // dd_FreePolyhedra(poly);
    // cout<<"Free poly."<<endl;
    // int h;
    // h = getchar();
    
    // dd_FreeMatrix(giwc_span_cdd);
    // cout<<"Free giwc_span_cdd."<<endl;
    // int i;
    // i = getchar();
    
    dd_free_global_constants();
    // cout<<"Free global constraint."<<endl;
    // int j;
    // j = getchar();

}

void CBirrtProblem::GetSupportPolygon(std::vector<string>& supportlinks, std::vector<dReal>& polyx, std::vector<dReal>& polyy, Vector polyscale, Vector polytrans)
{
    int numsupportlinks = supportlinks.size();

    std::vector<boost::shared_ptr<KinBody::Link::Geometry> >_listGeomProperties;
    std::vector<Vector> points;
    AABB bounds;
    //get points on trimeshes of support links
    vector<KinBody::LinkPtr> vlinks = robot->GetLinks();
    for(int i = 0 ; i < numsupportlinks; i++)
    {
        for(int j =0; j < vlinks.size(); j++)
        {
            if(strcmp(supportlinks[i].c_str(), vlinks[j]->GetName().c_str()) == 0 )
            {
                RAVELOG_DEBUG("Found match!\n");
                _listGeomProperties = vlinks[j]->GetGeometries();

                //compute AABBs for the link at identity
                for(int k = 0; k < _listGeomProperties.size(); k++)
                {
                    //if( _listGeomProperties.size() == 1){
                    Transform _t = _listGeomProperties[k]->GetTransform().inverse();
                    //bounds = _listGeomProperties[k]->ComputeAABB(_t);
                    bounds = _listGeomProperties[k]->ComputeAABB(_listGeomProperties[k]->GetTransform());
                    Transform offset = vlinks[j]->GetTransform()*_t;
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z + bounds.extents.z));
                }
                break;
            }
        }
    }
    // RAVELOG_INFO("Num support points in to qhull: %d\n",points.size());
    std::vector<coordT> pointsin(points.size()*2);
    std::vector<RaveVector<float> > plotvecs(points.size());
    for(int i = 0; i < points.size();i++)
    {
        pointsin[i*2 + 0] = points[i].x;
        pointsin[i*2 + 1] = points[i].y;
        plotvecs[i] = RaveVector<float>(points[i].x,points[i].y,points[i].z);
    }
    //GraphHandlePtr graphptr1 = GetEnv()->plot3(&plotvecs[0].x,plotvecs.size(),sizeof(plotvecs[0]),5, RaveVector<float>(1,0, 0, 1));
    //graphptrs.push_back(graphptr1);

    coordT* pointsOut = NULL;

    int numPointsOut = 0;

    convexHull2D(&pointsin[0], points.size(), &pointsOut, &numPointsOut);
    // RAVELOG_INFO("Num support points out of qhull:: %d\n",numPointsOut);
    
    std::vector<RaveVector<float> > tempvecs(numPointsOut +1);
    polyx.resize(numPointsOut);
    polyy.resize(numPointsOut);

    Point2D polygon[numPointsOut];
    dReal centerx = 0;
    dReal centery = 0;
    for(int i =0; i < numPointsOut; i++)
    {
        polyx[i] = pointsOut[(i)*2 + 0];
        polyy[i] = pointsOut[(i)*2 + 1];
        polygon[i].x = polyx[i];
        polygon[i].y = polyy[i];
        //centerx += polyx[i];
        //centery += polyy[i];
    }

    size_t vertexCount = sizeof(polygon) / sizeof(polygon[0]);
    Point2D centroid = compute2DPolygonCentroid(polygon, vertexCount);
    //std::cout << "Centroid is (" << centroid.x << ", " << centroid.y << ")\n";


    centerx = centroid.x;//centerx/numPointsOut;
    centery = centroid.y;//centery/numPointsOut;
    // RAVELOG_INFO("center %f %f\n",centerx, centery);

    for(int i =0; i < numPointsOut; i++)
    {
        polyx[i] = polyscale.x*(polyx[i] - centerx) + centerx + polytrans.x;
        polyy[i] = polyscale.y*(polyy[i] - centery) + centery + polytrans.y;
        tempvecs[i] = RaveVector<float>(polyx[i],polyy[i],0);
        // cout<<"("<<polyx[i]<<","<<polyy[i]<<")"<<endl;
    }

    

    //close the polygon
    tempvecs[tempvecs.size()-1] = RaveVector<float>(polyx[0],polyy[0],0);
    // GraphHandlePtr graphptr = GetEnv()->drawlinestrip(&tempvecs[0].x,tempvecs.size(),sizeof(tempvecs[0]),5, RaveVector<float>(0, 1, 1, 1));
    // graphptrs.push_back(graphptr);

    free(pointsOut);
}


bool CBirrtProblem::Traj(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    string filename; sinput >> filename;
    if( !sinput )
        return false;

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(), "");
    //ptraj->Init(robot->GetConfigurationSpecification());

    ifstream infile(filename.c_str(),ios::in);
    infile.precision(16);
    ptraj->deserialize(infile);

    RAVELOG_VERBOSE("executing traj\n");
    robot->GetController()->SetPath(ptraj);
    sout << "1";
    return true;

}




CBirrtProblem::Point2D CBirrtProblem::compute2DPolygonCentroid(const CBirrtProblem::Point2D* vertices, int vertexCount)
{
    Point2D centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;
    for (i=0; i<vertexCount-1; ++i)
    {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[i+1].x;
        y1 = vertices[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}



void CBirrtProblem::GetDistanceFromLineSegment(dReal cx, dReal cy, dReal ax, dReal ay,  dReal bx, dReal by, dReal& distanceSegment, dReal& xout, dReal& yout)
{

    dReal distanceLine;

    dReal r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
    dReal r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
    dReal r = r_numerator / r_denomenator;

    dReal px = ax + r*(bx-ax);
    dReal py = ay + r*(by-ay);

    dReal s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;

    distanceLine = fabs(s)*sqrt(r_denomenator);

    dReal xx = px;
    dReal yy = py;

    if ( (r >= 0) && (r <= 1) )
    {
        distanceSegment = distanceLine;
    }
    else
    {

        dReal dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
        dReal dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
        if (dist1 < dist2)
        {
            xx = ax;
            yy = ay;
            distanceSegment = sqrt(dist1);
        }
        else
        {
            xx = bx;
            yy = by;
            distanceSegment = sqrt(dist2);
        }


    }
    xout = xx;
    yout = yy;
}

bool CBirrtProblem::GetCamView(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    Transform camtm = GetEnv()->GetViewer()->GetCameraTransform();
    sout << camtm.rot.x << " " << camtm.rot.y << " " << camtm.rot.z << " " << camtm.rot.w << " " << camtm.trans.x << " " << camtm.trans.y << " " << camtm.trans.z;
    return true;
}

bool CBirrtProblem::SetCamView(ostream& sout, istream& sinput)
{
    //lock mutex
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());

    Transform camtm;
    sinput >> camtm.rot.x;
    sinput >> camtm.rot.y;
    sinput >> camtm.rot.z;
    sinput >> camtm.rot.w;
    sinput >> camtm.trans.x;
    sinput >> camtm.trans.y;
    sinput >> camtm.trans.z;
    
    GetEnv()->GetViewer()->SetCamera(camtm);
    return true;
}

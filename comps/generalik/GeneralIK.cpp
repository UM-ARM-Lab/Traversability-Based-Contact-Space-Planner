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
/** \file GeneralIK.cpp
    \brief Implements the generalik class.
 */
#include "stdafx.h"
#include "GeneralIK.h"


const int INF = 1000000;


GeneralIK::GeneralIK(EnvironmentBasePtr penv) : IkSolverBase(penv)
{
    __description = ":Interface Author: Dmitry Berenson\nAn iterative IK solver for kinematic chains. Can also take into account balance constraints. \n\n`C++ Documentation <http://automation.berkeley.edu/~berenson/docs/generalik/index.html>`_";

    _pEnvironment = penv; 
}


bool GeneralIK::Init(const RobotBase::ManipulatorPtr pmanip)
{
    _pmanip = pmanip;
    _pRobot = _pmanip->GetRobot();

    std::vector<KinBodyPtr> bodies;
    _pEnvironment->GetBodies(bodies);
    _pObstacle = bodies;
    
    bPRINT = false;
    bDRAW = false;
    bWRITETRAJ = false;
    bQUAT = false;
    bTRANSLATION_ONLY = false;
    bOBSTACLE_AVOIDANCE = false;


    RAVELOG_INFO("Initializing GeneralIK Solver\n");

    _oldnumdofs = -1;
    _numdofs = -1;
    _oldnumtargdims = -1;
    _numtargdims = -1;

    _tasktm.ReSize(3,3);
    _E_rpy.ReSize(3,3);
    _E_rpy_inv.ReSize(3,3);

    balancedx.ReSize(3);
    Mbal.ReSize(3);
    Mbalinv.ReSize(3);

    Mbalperp.ReSize(3);
    Mbalperpinv.ReSize(3);

    curquat.ReSize(4);
    angveltoquat.ReSize(4,4);

    balance_mode = BALANCE_NONE;

    return true;
}


void GeneralIK::ResizeMatrices()
{
    J.ReSize(_numtargdims,_numdofs);

    Jtrans.ReSize(_numtransdims,_numdofs);

    Jtemp.ReSize(_dimspergoal,_numdofs);
    Jplus.ReSize(_numdofs,_numtargdims);
    dx.ReSize(_numtargdims);
    dx_trans.ReSize(_numtransdims);
    step.ReSize(_numdofs);
    nullspacestep.ReSize(_numdofs);
    obstacleavoidancestep.ReSize(_numdofs);

    M.ReSize(_numtargdims);
    Minv.ReSize(_numtargdims);

    q_s_old.resize(_numdofs);
    _curtms.resize(_targtms.size());

    _Jp.ReSize(3,_numdofs);
    _Jp0.ReSize(3,_numdofs);
    _Jr.ReSize(3,_numdofs);
    _Jr0.ReSize(3,_numdofs);
    _Jr_proper.ReSize(3,_numdofs);
    q_limits.ReSize(_numdofs);

    Jtemp2.ReSize(3,_numdofs);
    Jtemp3.ReSize(3,_numdofs);

    _Jr_quat.ReSize(4,_numdofs);

    _S.ReSize(_numtargdims);
    _V.ReSize(_numtargdims,_numtargdims);
}


bool GeneralIK::Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
{
    RobotBase::ManipulatorConstPtr activemanip_p = _pRobot->GetActiveManipulator();

    dReal* qResult = &(*result.get())[0];
    const dReal* pFreeParameters = &vFreeParameters[0];
    solutionpath.resize(0);
    _targmanips.resize(0);
    _targtms.resize(0);
    movementlimit = INF;
    
    bool bsuccess = true;
    _pRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);
    
    int numdofs = _pRobot->GetActiveDOF();

    std::vector<dReal> q_s(numdofs);

    for(int i = 0; i < numdofs; i++)
    {
        q_s[i] = q0[i];
    }

    //set joint velocity weight (the lower the faster)
    // W.ReSize(numdofs);
    // Winv.ReSize(numdofs);
    // W = 1.0;
    // Winv = 1.0;
    // for(int i = 0; i < numdofs; i++)
    // {
    //     int DOF_index = _pRobot->GetActiveDOFIndices()[i];
    //     string DOF_name = _pRobot->GetJoints()[DOF_index]->GetName();

    //     // if(DOF_name == "l_knee_pitch" || DOF_name == "l_ankle_pitch" || DOF_name == "l_hip_pitch")
    //     // {
    //     //     W(i+1) = 0.1;
    //     // }
    //     // else if(DOF_name == "r_knee_pitch" || DOF_name == "r_ankle_pitch" || DOF_name == "r_hip_pitch")
    //     // {
    //     //     W(i+1) = 0.1;
    //     // }
    //     // else
    //     // {
    //     //     W(i+1) = 10.0;
    //     // }

    //     // if(DOF_name == "l_shoulder_pitch" || DOF_name == "l_shoulder_roll" || DOF_name == "l_shoulder_yaw" || DOF_name == "l_elbow_pitch" || DOF_name == "l_elbow_roll" || DOF_name == "l_wrist_pitch" || DOF_name == "l_wrist_yaw")
    //     // {
    //     //     W(i+1) = 100.0;
    //     // }
    //     // else if(DOF_name == "r_shoulder_pitch" || DOF_name == "r_shoulder_roll" || DOF_name == "r_shoulder_yaw" || DOF_name == "r_elbow_pitch" || DOF_name == "r_elbow_roll" || DOF_name == "r_wrist_pitch" || DOF_name == "r_wrist_yaw")
    //     // {
    //     //     W(i+1) = 100.0;
    //     // }
    //     // else
    //     // {
    //     //     W(i+1) = 0.001;
    //     // }

    //     if(DOF_name == "roll_revolute_joint" || DOF_name == "pitch_revolute_joint" || DOF_name == "yaw_revolute_joint" || DOF_name == "waist_yaw")
    //     {
    //         W(i+1) = 0.5;
    //     }


    //     // if(DOF_name == "waist_yaw")
    //     // {
    //     //     W(i+1) = 0.01;
    //     // }
    //     // else
    //     // {
    //     //     W(i+1) = 10.0;
    //     // }

    //     Winv(i+1) = 1 / W(i+1);
    // }

    // for(int i = 0; i < _numdofs; i++)
    // {
    //     if(fabs(q_s[i] - _lowerLimit[i]) < 0.0001 || fabs(q_s[i] - _upperLimit[i]) < 0.0001)
    //     {
    //         W(i+1) = 0.25 * (_upperLimit[i]-_lowerLimit[i]) / 0.0001;
    //     }
    //     else
    //     {
    //         W(i+1) = 0.25 * pow((_upperLimit[i]-_lowerLimit[i]),2) / ((q_s[i] - _lowerLimit[i]) * (_upperLimit[i]-q_s[i]));
    //     }

    //     Winv(i+1) = 1 / W(i+1);
    // }

    //read in the ik targets

    int numtargets = (int) pFreeParameters[0];

    for(int i = 0; i < numtargets; i++)
    {
        _targmanips.push_back(pFreeParameters[i*8 + 1]);
        _targtms.push_back(Transform(Vector(pFreeParameters[i*8 + 2],pFreeParameters[i*8 + 3],pFreeParameters[i*8 + 4],pFreeParameters[i*8 + 5]),Vector(pFreeParameters[i*8 + 6],pFreeParameters[i*8 + 7],pFreeParameters[i*8 + 8]) ));
        if(bPRINT)
             RAVELOG_INFO("Targtm: %f %f %f %f    %f %f %f\n",_targtms[i].rot.x,_targtms[i].rot.y,_targtms[i].rot.z,_targtms[i].rot.w,_targtms[i].trans.x,_targtms[i].trans.y,_targtms[i].trans.z);
    }

    int offset = numtargets*8 + 1;

    //read in obstacle vector
    bOBSTACLE_AVOIDANCE = (bool) pFreeParameters[offset++];
    if(bOBSTACLE_AVOIDANCE)
    {
        int checkcollisionlink_number = (int) pFreeParameters[offset++];
        for(int i  = 0; i < checkcollisionlink_number; i++)
        {
            int link_index = (int) pFreeParameters[offset++];
            KinBody::LinkPtr link = _pRobot->GetLinks()[link_index];
            control_points.insert(std::pair<string,Vector>(link->GetName(),link->GetCOMOffset()));
        }

        int checkselfcollisionlinkpair_number = (int) pFreeParameters[offset++];
        for(int i  = 0; i < checkselfcollisionlinkpair_number; i++)
        {
            int link_index_1 = (int) pFreeParameters[offset++];
            int link_index_2 = (int) pFreeParameters[offset++];
            KinBody::LinkPtr link_1 = _pRobot->GetLinks()[link_index_1];
            KinBody::LinkPtr link_2 = _pRobot->GetLinks()[link_index_2];
            self_collision_checking_pairs.push_back(std::pair<string,string>(link_1->GetName(),link_2->GetName()));
        }        

        int obstacle_number = (int) pFreeParameters[offset++];
        std::vector<KinBodyPtr> tempobstacle;
        for(int i = 0; i < obstacle_number; i++)
        {
            int obstacle_index = (int) pFreeParameters[offset++];
            tempobstacle.push_back(_pObstacle[obstacle_index]);

            Vector temp_repulsive_vector;
            temp_repulsive_vector.x = pFreeParameters[offset++];
            temp_repulsive_vector.y = pFreeParameters[offset++];
            temp_repulsive_vector.z = pFreeParameters[offset++];

            obstacle_repulsive_vector.push_back(temp_repulsive_vector);
        }
        _pObstacle = tempobstacle;


        // CollisionCheckerBasePtr pchecker;

        // pchecker = RaveCreateCollisionChecker(_pEnvironment,"pqp");
        // pchecker->SetCollisionOptions(CO_Contacts|CO_Distance);

        // _pEnvironment->SetCollisionChecker(pchecker);
    }

    //read in cog target if there is one
    int support_mode = (int) pFreeParameters[offset++];
    if(support_mode == 1)
    {
        if (bPRINT)
            RAVELOG_INFO("Support mode: Support polygon \n");

        cogtarg.x = pFreeParameters[offset++];
        cogtarg.y = pFreeParameters[offset++];
        cogtarg.z = pFreeParameters[offset++];
    
        if(bPRINT)
            RAVELOG_INFO("cog target: %f %f %f\n",cogtarg.x,cogtarg.y,cogtarg.z);    

        //read in the support polygon
        int numpoints = pFreeParameters[offset++];
        //int offset = numtargets*8 + 6;

        _vsupportpolyx.resize(numpoints);
        _vsupportpolyy.resize(numpoints);
        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyx[i] = pFreeParameters[offset++];
        }

        //offset = numtargets*8 + 6 + numpoints;

        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyy[i] = pFreeParameters[offset++];
        }


        for(int i = 0; i < _vsupportpolyy.size(); i++)
        {
            if(bPRINT)
                RAVELOG_INFO("x: %f    y:%f\n",_vsupportpolyx[i],_vsupportpolyy[i]);
        }

        balance_mode = BALANCE_SUPPORT_POLYGON;
    }
    else if(support_mode == 2)
    {
        if (bPRINT)
            RAVELOG_INFO("Support mode: Gravito-Inertial Wrench Cone\n");

        gravity.x = pFreeParameters[offset++];
        gravity.y = pFreeParameters[offset++];
        gravity.z = pFreeParameters[offset++];

        int cog_specified = pFreeParameters[offset++];

        cogtarg.x = pFreeParameters[offset++];
        cogtarg.y = pFreeParameters[offset++];
        cogtarg.z = pFreeParameters[offset++];

        int rowsize = (int) pFreeParameters[offset++];

        giwc.ReSize(rowsize, 6);
        // NOTE: 1-indexed
        for (int r = 1; r <= rowsize; r++) {
            for (int c = 1; c <= 6; c++) {
                giwc(r, c) = pFreeParameters[offset++];
            }
        }

        // sample around cog to find afeasible cog
        if(cog_specified == 0)
        {
            bool valid_goal_found = false;
            // if the cog goal does not make the robot in balance
            if(!CheckSupport(cogtarg))
            {
                for(float tmp_r = 0.01; tmp_r <= 0.5; tmp_r = tmp_r+0.01)
                {
                    for(float tmp_theta = 0; tmp_theta <= 360; tmp_theta = tmp_theta+10)
                    {
                        float tmp_cog_x = cogtarg.x + tmp_r * sin(tmp_theta*M_PI/180.0);
                        float tmp_cog_y = cogtarg.x + tmp_r * cos(tmp_theta*M_PI/180.0);
                        if(CheckSupport(Vector(tmp_cog_x,tmp_cog_y,cogtarg.z)))
                        {
                            cogtarg.x = tmp_cog_x;
                            cogtarg.y = tmp_cog_y;
                            valid_goal_found = true;
                            break;
                        }
                    }

                    if(valid_goal_found)
                    {
                        break;
                    }
                }
            }
        }

        balance_mode = BALANCE_GIWC;
    }
    else
    {
        balance_mode = BALANCE_NONE;
    }


    // if(support_mode == 2)
    // {
    //     // std::vector<float> green;
    //     _pRobot->SetActiveDOFValues(q_s);
    //     Vector robot_center_transform = 0.5 * (_pRobot->GetLink("l_foot")->GetTransform().trans + _pRobot->GetLink("r_foot")->GetTransform().trans);
    //     robot_center_transform.z = robot_center_transform.z + 0.9;

    //     float sampling_range_xy = 0.3;
    //     float sampling_range_z = 0.2;

    //     Vector tmp_cogtarg(0,0,0);
    //     int feasible_cog_count = 0;

    //     // cout<<cogtarg<<endl;

    //     for(dReal xi = cogtarg.x - sampling_range_xy; xi <= cogtarg.x + sampling_range_xy; xi = xi + 0.02)
    //     {
    //         for(dReal yi = cogtarg.y - sampling_range_xy; yi <= cogtarg.y + sampling_range_xy; yi = yi + 0.02)
    //         {
    //             for(dReal zi = robot_center_transform.z - sampling_range_z; zi <= robot_center_transform.z + sampling_range_z; zi = zi + 0.02)
    //             {
    //                 // cout<<Vector(xi,yi,zi);
    //                 if(CheckSupport(Vector(xi,yi,zi)))
    //                 {
    //                     tmp_cogtarg.x += xi;
    //                     tmp_cogtarg.y += yi;
    //                     tmp_cogtarg.z += zi;
    //                     feasible_cog_count += 1;
    //                     // green.push_back(xi);
    //                     // green.push_back(yi);
    //                     // green.push_back(zi);
    //                 }

    //                 // if (green.size() > 0)
    //                 //     graphptrs.push_back(GetEnv()->plot3(&green.front(), green.size()/3, 3, 5, Vector(0, 1, 0, 1)));
    //                 // green.clear();

    //             }
    //         }
    //     }

    //     if(feasible_cog_count != 0)
    //     {
    //         tmp_cogtarg.x /= feasible_cog_count;
    //         tmp_cogtarg.y /= feasible_cog_count;
    //         tmp_cogtarg.z /= feasible_cog_count;
    //         cogtarg = tmp_cogtarg;
    //     }
    //     // cout<<cogtarg<<endl;

    //     // getchar();
    //     // graphptrs.clear();
        
    // }


    //this used to be a mode but is currently ignored
    int junk = (int)pFreeParameters[offset++]; //mode = (GeneralIK::Mode)pFreeParameters[offset++];
    bTRANSLATION_ONLY = (bool)pFreeParameters[offset++];
    bEXACT_COG = (bool)pFreeParameters[offset++];


    if(bEXACT_COG)
    {
        cogtarg.x = pFreeParameters[offset++];
        cogtarg.y = pFreeParameters[offset++];
        cogtarg.z = pFreeParameters[offset++];
    }
    
    if(vFreeParameters.size()-1 == offset)
    {
        movementlimit = pFreeParameters[offset];
    }

    if(bTRANSLATION_ONLY)
    {
        _dimspergoal = 3;
        _numrotdims = 0;
    }
    else
    {
        if(bQUAT)
        {
            _dimspergoal = 7;
            _numrotdims = _targtms.size()*4;
        }
        else
        {
            _dimspergoal = 6;
            _numrotdims = _targtms.size()*3;
        }
    }

    _numtransdims = _targtms.size()*3;
    _numtargdims = _targtms.size()*_dimspergoal;
    //RAVELOG_INFO("extratarg = %f\n",pFreeParameters[numtargets*8 + 1]);


    _numdofs = numdofs;
    

    if( (_oldnumdofs != _numdofs) || (_oldnumtargdims != _numtargdims) )
    {
        ResizeMatrices();
        _oldnumdofs = _numdofs;
        _oldnumtargdims = _numtargdims;
    }

    if(bWRITETRAJ)
    {
        ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_pRobot->GetActiveConfigurationSpecification());
        //_trajpoint.q.resize(_pRobot->GetActiveDOF());
        //_trajpoint.qdot.resize(_pRobot->GetActiveDOF());
    }

    if(!CheckDOFValueIntegrity(q_s))
    {
        RAVELOG_INFO("ERROR: DOF values are incorrect, cannot start IK solver!\n");
        return false;
    }

    bsuccess  = _SolveStopAtLimits(q_s);

    //always copy the joint vals into q_s (this could be the closest the robot could get)
    *result.get() = q_s;

    giwc.ReleaseAndDelete();

    if(bDRAW)
        DrawSolutionPath();

    if(bPRINT)
        RAVELOG_INFO("Number of iterations: %d x_error: %f\n",_numitr,x_error);

    if(bWRITETRAJ)
        WriteTraj();

    _pRobot->SetActiveManipulator(activemanip_p);

    return (bsuccess);

}





dReal GeneralIK::TransformDifferenceVectorized(dReal * dx,std::vector<Transform>& tm_refs, std::vector<Transform>& tm_targs)
{


    for(int i = 0; i < tm_refs.size(); i++)
    {
        TransformDifference(dx + i*_dimspergoal,tm_refs[i],tm_targs[i]);
    }

    _sumsqr2 = 0;
    for(int i = 0; i < tm_refs.size()*_dimspergoal; i++)
        _sumsqr2 += dx[i]*dx[i];

    return sqrt(_sumsqr2);
}


dReal GeneralIK::TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ)
{
    _tmtemp = tm_ref.inverse()*tm_targ;

    dx[0] = _tmtemp.trans.x;
    dx[1] = _tmtemp.trans.y;
    dx[2] = _tmtemp.trans.z;

    if(bTRANSLATION_ONLY)
    {
        //do nothing
    }
    else
    {
        if(bQUAT)
        {
            //check which version of quaternion is closer
            _q1 = tm_targ.rot - tm_ref.rot;
            _q2 = -tm_targ.rot - tm_ref.rot;
            if(_q1.lengthsqr4() < _q2.lengthsqr4())
            {
                dx[3] = _q1.x;
                dx[4] = _q1.y;
                dx[5] = _q1.z;
                dx[6] = _q1.w;
            }
            else
            {
                dx[3] = _q2.x;
                dx[4] = _q2.y;
                dx[5] = _q2.z;
                dx[6] = _q2.w;
            }
        }
        else
            QuatToRPY(_tmtemp,dx[3],dx[4],dx[5]);
    }

    _sumsqr = 0;
    for(int i = 0; i < _dimspergoal; i++)
        _sumsqr += dx[i]*dx[i];

    // RAVELOG_INFO("dx: %f %f %f %f %f %F\n",dx[0],dx[1],dx[2],dx[3],dx[4],dx[5]);

    return sqrt(_sumsqr);
}


void GeneralIK::GetFullJacobian(Transform tEE, Transform taskframe_in, NEWMAT::Matrix& J)
{

    std::vector<dReal> temp;

     _pRobot->CalculateActiveJacobian(_pRobot->GetActiveManipulator()->GetEndEffector()->GetIndex(), tEE.trans, temp);
    //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");
    memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

    _pRobot->CalculateActiveAngularVelocityJacobian(_pRobot->GetActiveManipulator()->GetEndEffector()->GetIndex(), temp);
    memcpy(_Jr0.Store(),&temp[0],temp.size()*sizeof(dReal));
    
    //PrintMatrix(_Jr0.Store(),3,_numdofs,"Jr0: ");

    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

    _Jp = _tasktm * _Jp0;

    if(bTRANSLATION_ONLY)
    {
        J = _Jp;
        return;
    }

    _Jr = _tasktm * (-_Jr0);

    //PrintMatrix(_tasktm.Store(),3,3,"tasktm: ");
    //PrintMatrix(_Jp.Store(),3,_numdofs,"Jp: ");
    //PrintMatrix(_Jr.Store(),3,_numdofs,"Jr: ");


    if(bQUAT)
    {
        Vector vcurquat = (taskframe_in.inverse()*tEE).rot;
        NEWMAT::ColumnVector curquat(4);
        curquat(1) = vcurquat.x;
        curquat(2) = vcurquat.y;
        curquat(3) = vcurquat.z;
        curquat(4) = vcurquat.w;


        NEWMAT::Matrix angveltoquat(4,4);
        
        for(int i =1; i <= _numdofs; i++)
        {   
            //get correction matrix using angular velocity from Jr
            angveltoquat(1,1) = 0;          angveltoquat(1,2) = -_Jr(1,i);    angveltoquat(1,3) = -_Jr(2,i);  angveltoquat(1,4) = -_Jr(3,i);
            angveltoquat(2,1) = _Jr(1,i);   angveltoquat(2,2) = 0;            angveltoquat(2,3) = -_Jr(3,i);  angveltoquat(2,4) = _Jr(2,i);
            angveltoquat(3,1) = _Jr(2,i);   angveltoquat(3,2) = _Jr(3,i);     angveltoquat(3,3) = 0;          angveltoquat(3,4) = -_Jr(1,i);
            angveltoquat(4,1) = _Jr(3,i);   angveltoquat(4,2) = -_Jr(2,i);    angveltoquat(4,3) = _Jr(1,i);   angveltoquat(4,4) = 0;

            _Jr_quat.Column(i) = 0.5 * (angveltoquat * curquat);
        }

        //PrintMatrix(Jr_proper.Store(),3,_numdofs,"Jr_proper: ");

        J = _Jp & -_Jr_quat;
    }
    else
    {

        //convert current rotation to euler angles (RPY) 
        QuatToRPY(taskframe_in.inverse()*tEE,_psi,_theta,_phi);    
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(_phi);
        Ctheta = cos(_theta);
        Cpsi = cos(_psi);

        Sphi = sin(_phi);
        Stheta = sin(_theta);
        Spsi = sin(_psi);

        _E_rpy(1,1) = Cphi/Ctheta;         _E_rpy(1,2) = Sphi/Ctheta;         _E_rpy(1,3) = 0;
        _E_rpy(2,1) = -Sphi;               _E_rpy(2,2) = Cphi;                _E_rpy(2,3) = 0;
        _E_rpy(3,1) = Cphi*Stheta/Ctheta;  _E_rpy(3,2) = Sphi*Stheta/Ctheta;  _E_rpy(3,3) = 1;


        //PrintMatrix(E_rpy.Store(),3,3,"E_rpy: ");

        _Jr_proper = _E_rpy * _Jr;

        //PrintMatrix(Jr_proper.Store(),3,_numdofs,"Jr_proper: ");

        J = _Jp & -_Jr_proper;

        //PrintMatrix(J.Store(),3,_numdofs,"J1: ");
    }

}

void GeneralIK::GetCOGJacobian(Transform taskframe_in, NEWMAT::Matrix& J, Vector& center)
{


    J = 0.0;

    center = Vector(0,0,0,0);
    dReal fTotalMass = 0;
    std::vector<KinBody::LinkPtr>::const_iterator itlink;



    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];


    //PrintMatrix(E_rpy.Store(),3,3,"E_rpy: ");

    std::vector<dReal> temp;
    FORIT(itlink, _pRobot->GetLinks()) {
        //RAVELOG_INFO("comoffset: %f %f %f\n", (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
        //GetEnv()->plot3((*itlink)->GetTransform() * (*itlink)->GetCOMOffset(), 1, 0, 0.01*(*itlink)->GetMass(), Vector(1,0,0),1);

        _pRobot->CalculateActiveJacobian((*itlink)->GetIndex(), ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset()), temp);
        //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");
        memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));



        _Jp = _tasktm * _Jp0;

        //RAVELOG_INFO("Link name: %s, mass: %f\n",(*itlink)->GetName().c_str(),(*itlink)->GetMass());
        //PrintMatrix(_tasktm.Store(),3,3,"tasktm: ");
        //PrintMatrix(_Jp.Store(),3,_numdofs,"Jp: ");
        //PrintMatrix(_Jr.Store(),3,_numdofs,"Jr: ");

        J = J + ((*itlink)->GetMass()*(_Jp.Rows(1,3)));


        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        center /= fTotalMass;
    // RAVELOG_INFO("\nmass: %f\ncog: %f %f %f\n",fTotalMass,center.x,center.y,center.z);
   
    J = J/fTotalMass;

}

void GeneralIK::GetOAJacobian(Transform taskframe_in, NEWMAT::Matrix& J, std::multimap<string,Vector>::iterator& control_point)
{

    //calculate the velocity of each control point
    //calculate jacobian for each control point to generate the joint angular velocity
    //sum the joint angular velocity

    //center = Vector(0,0,0,0);
    dReal fTotalMass = 0;
    //std::vector<KinBody::LinkPtr>::const_iterator itlink;
    

    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

    KinBody::LinkPtr target_link = _pRobot->GetLink(control_point->first);
    std::vector<dReal> temp;

    _pRobot->CalculateActiveJacobian(target_link->GetIndex(), (target_link->GetTransform() * control_point->second), temp);
    memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

    _Jp = _tasktm * _Jp0;

    J = _Jp;
}



void GeneralIK::PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement)
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


Vector GeneralIK::RPYIdentityOffsets[8] = { Vector(M_PI,M_PI,M_PI),
                                            Vector(M_PI,M_PI,-M_PI),
                                            Vector(M_PI,-M_PI,M_PI),
                                            Vector(M_PI,-M_PI,-M_PI),
                                            Vector(-M_PI,M_PI,M_PI),
                                            Vector(-M_PI,M_PI,-M_PI),
                                            Vector(-M_PI,-M_PI,M_PI),
                                            Vector(-M_PI,-M_PI,-M_PI)};

void GeneralIK::QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi)
{
    
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
   _min_dist = 10000;
    for(int i =0; i < 9; i++)
    {

        if(i == 0) //for first element, use original values
        {
            _temp_vec.x = psi;
            _temp_vec.y = theta;
            _temp_vec.z = phi;
        }
        else
        {
            _temp_vec.x = psi + RPYIdentityOffsets[i-1].x;
            _temp_vec.y = -theta + RPYIdentityOffsets[i-1].y;//note that theta is negative
            _temp_vec.z = phi + RPYIdentityOffsets[i-1].z;
        }
        
        _temp_dist = _temp_vec.lengthsqr3();
        if(_temp_dist < _min_dist)
        {
            _min_dist = _temp_dist;
            psi = _temp_vec.x;
            theta = _temp_vec.y;
            phi = _temp_vec.z;
        }
    }


    //RAVELOG_INFO("psi: %f, theta: %f, phi: %f\n",psi,theta,phi);
}


bool GeneralIK::CheckSupport(Vector center)
{
    bool balanced = false;
    if (balance_mode == BALANCE_SUPPORT_POLYGON) {
        int nvert = _vsupportpolyx.size();
        if(nvert == 0)
            return false;


        dReal testx = center.x;
        dReal testy = center.y;
        dReal * vertx = &_vsupportpolyx[0];
        dReal * verty = &_vsupportpolyy[0];


        int i, j;
        for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
         (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
           balanced = !balanced;
        }

        center.z = 0;
    } else if (balance_mode == BALANCE_GIWC) {

        Vector crossprod = center.cross(gravity);

        NEWMAT::ColumnVector giwc_test_vector(6);
        giwc_test_vector << gravity.x << gravity.y << gravity.z
                         << crossprod.x << crossprod.y << crossprod.z;
        
        NEWMAT::ColumnVector result = giwc * giwc_test_vector;

        balanced = true;
        // double min_value = INF;
        // Test to see if any item in the result is less than 0
        // NOTE: 1-indexed
        for (int i = 1; i <= result.Nrows(); i++) {
            if (result(i) < 0) {
                balanced = false;
                break;
            }

            // if(result(i) < min_value)
            // {
            //     min_value = result(i);
            // }
        }
        // cout<<min_value<<endl;

        if(bDRAW)
            graphptrs.push_back(GetEnv()->plot3(&(DoubleVectorToFloatVector(cogtarg)[0]), 1, 0, 10, Vector(0, 0, 1) ));

    }
    solutionpath.push_back(center);
    // RAVELOG_INFO("cog: %f %f %f\n", center.x,center.y,center.z);
    if(balanced)
    {
        if (bPRINT)
            RAVELOG_INFO("Balance check: Balanced \n");
        if(bDRAW)
            graphptrs.push_back(GetEnv()->plot3(&(DoubleVectorToFloatVector(center)[0]), 1, 0, 10, Vector(0,1,0) ));
        return true;
    }
    else
    {
        if(bPRINT)
            RAVELOG_INFO("Balance check: Not balanced \n");
        if(bDRAW)
            graphptrs.push_back(GetEnv()->plot3(&(DoubleVectorToFloatVector(center)[0]), 1, 0, 10, Vector(1,0,0) ));
        return false;
    }
}


int GeneralIK::invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{
    int didfix = 0;
    NEWMAT::EigenValues(A,_S,_V);
    // Find the maximum eigenvalue
    dReal maxEig = 0;
    dReal minEig = 0;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e > maxEig) maxEig = e;
        if (i == 1 || e < minEig) minEig = e;
    }
    //RAVELOG_INFO("min/max eigenvalue: %f/%f\n", minEig, maxEig);

    dReal minEigDesired = maxEig/maxConditionNumber;
    int notfixcount = 0;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e < minEigDesired) {e = minEigDesired; didfix = 1; if(bPRINT) RAVELOG_INFO("MIN EIG COND FIX!\n");}
        else
            notfixcount++;
            
        if (maxEig > 100) { e = e/maxEig*100; if(bPRINT) RAVELOG_INFO("MAX EIG COND FIX!\n");}
        _S(i) = e;
    }
    if(bPRINT) RAVELOG_INFO("notfixcount: %d\n",notfixcount);
    //this just reconstructs the A matrix with better conditioning
    //Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();
    return didfix;
}

void GeneralIK::invConditioning(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{

    NEWMAT::EigenValues(A,_S,_V);
    // Find the maximum eigenvalue
    dReal maxEig = 0;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e > maxEig) maxEig = e;
    }
    //RAVELOG_INFO("max eigenvalue: %f\n", maxEig);

    dReal minEigDesired = maxEig/maxConditionNumber;
    for (int i = 1; i <= _S.Nrows(); ++i){
        dReal e = _S(i);
        if (e < minEigDesired) e = minEigDesired;
        //if (maxEig > 100) e = e/maxEig*100;
        _S(i) = e;
    }

    //this just reconstructs the A matrix with better conditioning
    //Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();

}

void GeneralIK::DrawSolutionPath()
{

    int pathlength = solutionpath.size();

    if(pathlength == 0)
        return;

    //float solutionpath0_x_float = (float)solutionpath[0].x;
    //graphptrs.push_back(GetEnv()->drawlinestrip(&solutionpath0_x_float,pathlength,sizeof(solutionpath[0]),3, RaveVector<float>(0, 0, 1, 0)));
    //graphptrs.push_back(GetEnv()->drawlinestrip(&solutionpath0_x_float,pathlength,sizeof(RaveVector<float>(0, 0, 1, 0)),3, RaveVector<float>(0, 0, 1, 0)));
    std::vector<RaveVector<float> > fSolutionPath;
    for(int i =0; i < solutionpath.size(); i++)
        fSolutionPath.push_back(RaveVector<float>(solutionpath[i].x,solutionpath[i].y,solutionpath[i].z));

    RaveVector<float> vblue = RaveVector<float>(0, 0, 1, 0);
    // graphptrs.push_back(GetEnv()->drawlinestrip(&(fSolutionPath[0].x),pathlength,sizeof(vblue),3, vblue));

    // graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(solutionpath[pathlength-1])[0]), 1, 0, 15, Vector(1,1,0) ));
    RAVELOG_INFO("cog at end: %f %f %f\n",solutionpath[pathlength-1].x,solutionpath[pathlength-1].y,solutionpath[pathlength-1].z);
}



void GeneralIK::GetClosestPointOnPolygon(Vector& point,Vector& closestpoint, Vector& perpvec)
{
    perpvec.z = 0;
    dReal xout,yout,dist;
    dReal mindist = 10000000;
    dReal endpntx,endpnty;

    for(int i = 0; i < _vsupportpolyx.size(); i++)
    {

        if( i == _vsupportpolyx.size()-1)
        {
            endpntx = _vsupportpolyx[0];
            endpnty = _vsupportpolyy[0];
        }
        else
        {
            endpntx = _vsupportpolyx[i+1];
            endpnty = _vsupportpolyy[i+1];
        }

        GetDistanceFromLineSegment(point.x, point.y, _vsupportpolyx[i], _vsupportpolyy[i],endpntx, endpnty, dist, xout, yout);

        if(dist < mindist)
        {
            mindist = dist;
            closestpoint.x = xout;
            closestpoint.y = yout;
            //get vector perpendicular to the edge
            perpvec.x = -(_vsupportpolyy[i] - endpnty);
            perpvec.y = _vsupportpolyx[i] - endpntx;

        }
    }

    perpvec = perpvec*(1/(sqrt(perpvec.x*perpvec.x + perpvec.y*perpvec.y)));

}


void GeneralIK::GetDistanceFromLineSegment(dReal cx, dReal cy, dReal ax, dReal ay ,
                      dReal bx, dReal by, dReal& distanceSegment,
                      dReal& xout, dReal& yout)
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

void GeneralIK::WriteTraj()
{
  
    //TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),"");
    //_pRobot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    /* new openrave added a fmaxaccelmult parameter (number 5) */
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, _pRobot,false,1,1,"LinearTrajectoryRetimer");
#else
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, _pRobot,false,1,"LinearTrajectoryRetimer");
#endif
    // save the constrained trajectory
    const char* filename2 = "iktraj.txt";
    //pfulltraj->CalcTrajTiming(_pRobot, pfulltraj->GetInterpMethod(), true, false);
    ofstream outfile(filename2,ios::out);
    ptraj->serialize(outfile);
    //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    outfile.close();
}


bool GeneralIK::CheckDOFValueIntegrity(const std::vector<dReal>& q_in)
{
    bool bok = true;
    for(int i = 0; i < _numdofs; i++)
    {
        if(q_in[i] < _lowerLimit[i] || q_in[i] > _upperLimit[i])
        {
            RAVELOG_INFO("ERROR: DOF %d has value %f, which is outside %f to %f\n",i,q_in[i],_lowerLimit[i],_upperLimit[i]);
            bok = false;
        }

        if(isnan(q_in[i]))
        {
            RAVELOG_INFO("ERROR: DOF %d is nan!\n",i);
            bok = false;
        }  
    }    
    return bok;
}


bool GeneralIK::_SolveStopAtLimits(std::vector<dReal>& q_s)
{

    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;
    badjointinds.resize(0);
    prev_error = 1000000.0;
    std::vector<dReal> q_s_backup = q_s;
    //initialize stepsize and epsilon
    maxstep = 0.1*_targtms.size();
    stepsize = maxstep;
    epsilon = 0.001;
    q_s_old = q_s;
    bClearBadJoints = true; //setting this to true will make the algorithm attempt to move joints that are at joint limits at every iteration

    std::map<string,Vector> control_points_in_collision;

    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(_targtms[i].trans)[0]), 1, 0, 5, Vector(0,1,0) ));
        }
    }

    // std::multimap<string,Vector> control_points;
    // if(bOBSTACLE_AVOIDANCE)
    // {
    //     ControlPointSampling(control_points);
    // }

    // for(std::vector<dReal>::iterator it = q_s.begin(); it != q_s.end(); it++)
    // {
    //     std::cout<<*it<<' ';
    // }
    // std::cout<<std::endl;

    bBalanceGradient = false;
    Vector perpvec;
    bool balanced;

    for(int kk = 0; kk < 200; kk++)
    {
        _numitr++;

        bLimit = false;

        // cout<<"Iteration: "<<kk<<endl;

        _pRobot->SetActiveDOFValues(q_s);
        if(bWRITETRAJ)
        {
            for(int i = 0; i < _pRobot->GetActiveDOF(); i++)
            {
                //_trajpoint.q[i] = q_s[i];
                ptraj->Insert(ptraj->GetNumWaypoints(),q_s,_pRobot->GetActiveConfigurationSpecification());
            }
            //ptraj->AddPoint(_trajpoint);
        }

        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(_curtms[i].trans)[0]), 1, 0, 5, Vector(1,0,0) ));
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");


        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);

        //if balance stuff is on, error will go up sometimes, so don't do this check

        if(balance_mode == BALANCE_NONE && (x_error >= prev_error || (prev_error - x_error < epsilon/10))&& x_error > epsilon)
        {
            if(bPRINT)
            {
                RAVELOG_INFO("no progress\n");
                RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);
            }
            stepsize = stepsize/2;
            x_error = prev_error;
            q_s = q_s_old;

        }
        else
            stepsize = maxstep;

        if(bPRINT)
            RAVELOG_INFO("stepsize: %f\n",stepsize);

        //don't let step size get too small

        // if(stepsize < epsilon)
        // {
        //     //if(bPRINT)
        //     //    RAVELOG_INFO("Projection stalled _numitr: %d\n",_numitr);
        //     RAVELOG_DEBUG("Projection stalled _numitr: %d\n",_numitr);
        //     return false;
        // }

        if(movementlimit != INF)
        {
            dReal qsnorm = 0;
            for(int i = 0; i < q_s.size(); i++)
            {
                qsnorm += (q_s[i] - q_s_backup[i])*(q_s[i] - q_s_backup[i]);
            }
            qsnorm = sqrt(qsnorm);
            if(qsnorm > movementlimit)
            {
                q_s = q_s_old;
                RAVELOG_INFO("Projection hit movement limit at itr %d\n",_numitr);
                return false;
            }
        }


        if(bClearBadJoints)
        {
            badjointinds.resize(0);
        }

        do{
            q_s_old = q_s;

            if(bLimit == false)
                prev_error = x_error;


            if(balance_mode != BALANCE_NONE || bEXACT_COG)
            {
               GetCOGJacobian(Transform(),Jtemp2,curcog);
            }

            balanced = (balance_mode == BALANCE_NONE || (CheckSupport(curcog)));
            // balanced = (balance_mode == BALANCE_NONE || (CheckSupport(curcog) && sqrt((curcog-cogtarg).lengthsqr2()) <= 0.2 && fabs(curcog.z-cogtarg.z) <= 0.1));
            // balanced = (balance_mode == BALANCE_NONE || (CheckSupport(curcog) && sqrt((curcog-cogtarg).lengthsqr2()) <= 0.2));

            //RAVELOG_INFO("xerror: %f\n",x_error);
            // if(x_error < epsilon && balanced && (bOBSTACLE_AVOIDANCE == false || control_points_in_collision.size() == 0)))
            if(x_error < epsilon && balanced && (bEXACT_COG == false || sqrt((curcog-cogtarg).lengthsqr2()) <= 0.05) && (bOBSTACLE_AVOIDANCE == false || control_points_in_collision.size() == 0))
            {
                if(bPRINT)
                    RAVELOG_INFO("Projection successfull _numitr: %d\n",_numitr);
                return true;
            }

            //only need to compute the jacobian once if there are joint limit problems
            if(bLimit == false)
            {

                for(int i = 0; i < _targmanips.size();i++)
                {
                    _pRobot->SetActiveManipulator(_targmanips[i]);
                    GetFullJacobian(_curtms[i],_targtms[i],Jtemp);
                    J.Rows(i*_dimspergoal +1,(i+1)*_dimspergoal) = Jtemp;
                }

                if(balance_mode != BALANCE_NONE || bEXACT_COG)
                {
                   GetCOGJacobian(Transform(),Jtemp2,curcog);

                   if(!balanced || (bEXACT_COG && sqrt((curcog-cogtarg).lengthsqr2()) > 0.05))
                   {
                        bBalanceGradient = true;
                        balancedx(1) = (curcog.x - cogtarg.x);
                        balancedx(2) = (curcog.y - cogtarg.y);
                        // balancedx(3) = (curcog.z - cogtarg.z);
                        balancedx(3) = 0;
                   }
                   else
                   {
                        bBalanceGradient = false;
                        balancedx(1) = 0;
                        balancedx(2) = 0;
                        balancedx(3) = 0;
                        // bBalanceGradient = true;
                        // balancedx(1) = (curcog.x - cogtarg.x);
                        // balancedx(2) = (curcog.y - cogtarg.y);
                        // balancedx(3) = (curcog.z - cogtarg.z);
                   }
                }
            }


            //eliminate bad joint columns from the Jacobian
            for(int j = 0; j < badjointinds.size(); j++)
                for(int k = 0; k < _numtargdims; k++)
                      J(k+1,badjointinds[j]+1) = 0;

            if(balance_mode != BALANCE_NONE || bEXACT_COG)
            {
                for(int j = 0; j < badjointinds.size(); j++)
                    for(int k = 0; k <3; k++)
                          Jtemp2(k+1,badjointinds[j]+1) = 0;
            }

            if(x_error > stepsize)
                magnitude = stepsize/x_error;
            else
                magnitude = 1;

            NEWMAT::DiagonalMatrix Reg(_numtargdims);
            NEWMAT::DiagonalMatrix Reg2(Jtemp2.Nrows());
            Reg = 0.0001;
            Reg2 = 0.0001;
            // Reg = 0.01;
            // Reg2 = 0.01;
            M << (J*J.t()) + Reg;
            // M << (J*Winv*J.t()) + Reg;

            invConditioningBound(10000,M,Minv);

            //Minv = M.i();
            //PrintMatrix(Minv.Store(),_numtargdims,_numtargdims,"Minv: ");

            Jplus = J.t()*Minv;
            // Jplus = Winv*J.t()*Minv;
            
            //PrintMatrix(Jplus.Store(),_numdofs,_numtargdims,"Jplus: ");

            //Add collision avoidance here
            //order: collision avoidance -> reach goal -> remain balance

            if(bOBSTACLE_AVOIDANCE)
            {
                obstacleavoidancestep.ReSize(_numdofs);
                obstacleavoidancestep = 0;
                NEWMAT::ColumnVector repulsive_vector_column;
                control_points_in_collision.clear();
                std::map<string,Vector> control_point_repulsive_vector;
                std::map<string,NEWMAT::Matrix> control_point_jacobian;
                for(std::map<string,Vector>::iterator ctrl_it = control_points.begin(); ctrl_it != control_points.end(); ctrl_it++)
                {
                    Vector repulsive_vector;
                    GetRepulsiveVector(repulsive_vector, ctrl_it);

                    if(repulsive_vector.lengthsqr3() != 0)
                    {
                        Jtemp3.ReSize(3,_numdofs);
                        GetOAJacobian(Transform(), Jtemp3, ctrl_it);
                        control_points_in_collision.insert(*ctrl_it);
                        control_point_repulsive_vector.insert(std::pair<string,Vector>(ctrl_it->first,repulsive_vector));
                        control_point_jacobian.insert(std::pair<string,NEWMAT::Matrix>(ctrl_it->first,Jtemp3));
                    }

                    // repulsive_vector_c(1) = repulsive_vector[0];
                    // repulsive_vector_c(2) = repulsive_vector[1];
                    // repulsive_vector_c(3) = repulsive_vector[2];
                    // obstacleavoidancestep = obstacleavoidancestep + (Jtemp3.t()*Moainv)*(1*repulsive_vector_c);
                }

                if(!control_points_in_collision.empty())
                {
                    //Jtemp3.CleanUp();
                    Jtemp3.ReSize(0,_numdofs);
                    repulsive_vector_column.ReSize(3*control_points_in_collision.size());

                    int point_index = 0;

                    //stack the repulsive vector and jacobian matrix
                    for(std::map<string,Vector>::iterator ctrl_it = control_points_in_collision.begin(); ctrl_it != control_points_in_collision.end(); ctrl_it++)
                    {
                        Jtemp3 &= control_point_jacobian.find(ctrl_it->first)->second;
                        repulsive_vector_column(point_index*3+1) = control_point_repulsive_vector.find(ctrl_it->first)->second.x;
                        repulsive_vector_column(point_index*3+2) = control_point_repulsive_vector.find(ctrl_it->first)->second.y;
                        repulsive_vector_column(point_index*3+3) = control_point_repulsive_vector.find(ctrl_it->first)->second.z;
                        point_index++;
                    }

                    for(int j = 0; j < badjointinds.size(); j++)
                        for(int k = 0; k < Jtemp3.Nrows(); k++)
                            Jtemp3(k+1,badjointinds[j]+1) = 0;

                    NEWMAT::DiagonalMatrix Reg3(Jtemp3.Nrows());
                    Reg3 = 0.0001;
                    Moa.ReSize(Jtemp3.Nrows());
                    Moainv.ReSize(Jtemp3.Nrows());
                    Moa << (Jtemp3*Jtemp3.t()) + Reg3;

                    invConditioningBound(10000,Moa,Moainv);
                    obstacleavoidancestep = (Jtemp3.t()*Moainv)*(0.03*repulsive_vector_column);
                    obstacleavoidancestep = obstacleavoidancestep / control_points_in_collision.size();
                }

                repulsive_vector_column.ReleaseAndDelete();
 
            }

            if(bBalanceGradient)
            {
                Mbal << (Jtemp2*Jtemp2.t()) + Reg2;
                invConditioningBound(10000,Mbal,Mbalinv);

                NEWMAT::Matrix Jtemp2plus = Jtemp2.t()*Mbalinv;

                //do ik, then move toward balance in null space
                if(bOBSTACLE_AVOIDANCE && !control_points_in_collision.empty())
                {
                    NEWMAT::Matrix Jtemp3plus = Jtemp3.t()*Moainv;
                    // nullspacestep = (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2plus*(1*balancedx) + (NEWMAT::IdentityMatrix(_numdofs) - Jtemp2plus*Jtemp2)*obstacleavoidancestep);
                    nullspacestep = (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp3plus*obstacleavoidancestep + (NEWMAT::IdentityMatrix(_numdofs) - Jtemp3plus*Jtemp3)*(1.0*balancedx));
                }
                else
                {
                    nullspacestep = (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*Jtemp2plus*(1.0*balancedx);
                }
                
            }
            else
            {
                if(bOBSTACLE_AVOIDANCE)
                {
                    nullspacestep = (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*obstacleavoidancestep;
                }
                else
                {
                    nullspacestep = 0;
                }
            }

            step = magnitude*Jplus*(dx) + nullspacestep;

            // cout<<"step: "<<endl;
            // cout<<step.t()<<endl;
            // cout<<"pose constraint step:"<<endl;
            // cout<<(step-nullspacestep).t()<<endl;
            // cout<<"nullspacestep: "<<endl;
            // cout<<nullspacestep.t()<<endl;
            // cout<<"obstacleavoidancestep: "<<endl;
            // cout<<obstacleavoidancestep.t()<<endl;
            // string hhh;
            // cin>>hhh;

            //RAVELOG_INFO("stepnorm: %f   nullspacestepnorm: %f\n", step.NormFrobenius(), nullspacestep.NormFrobenius());
            if(bPRINT)
                PrintMatrix(step.Store(),1,step.Nrows(),"step: ");

            //add step and check for joint limits
            bLimit = false;
            for(int i = 0; i < _numdofs; i++)
            {
                q_s[i] = q_s_old[i] - step(i+1);
                if(q_s[i] < _lowerLimit[i] || q_s[i] > _upperLimit[i])
                {
                    if(bPRINT)
                        RAVELOG_INFO("Jacobian going past joint limit. J%d: %f outside %f to %f\n",i,q_s[i],_lowerLimit[i],_upperLimit[i]);
                    
                    if(q_s[i] < _lowerLimit[i])
                        q_s[i] = _lowerLimit[i];
                    if(q_s[i] > _upperLimit[i])
                        q_s[i] = _upperLimit[i];

                    badjointinds.push_back(i); //note this will never add the same joint twice, even if bClearBadJoints = false
                    bLimit = true;

                }
            }

            //move back to previous point if any joint limits
            if(bLimit)
            {
                q_s = q_s_old;
            }

        }while(bLimit);


        if(bPRINT)
            RAVELOG_INFO("after limits\n");

    }

    // RAVELOG_INFO("Iteration limit reached\n");
    return false;

}


// void GeneralIK::ControlPointSampling(std::multimap<string,Vector>& control_points)
// {
//     //std::vector<string> sampled_links = ['l_foot','l_shin','l_thigh','r_foot','r_shin','r_thigh','torso','chest','l_upper_arm','l_forearm','r_upper_arm','r_forearm','head'];
//     //std::vector<string> sampled_links = ["l_foot","l_shin","r_foot","r_shin"];
//     std::vector<string> sampled_links;
//     sampled_links.push_back("l_foot");
//     sampled_links.push_back("l_shin");
//     sampled_links.push_back("l_thigh");
//     sampled_links.push_back("r_foot");
//     sampled_links.push_back("r_shin");
//     sampled_links.push_back("r_thigh");
//     sampled_links.push_back("l_palm");
//     sampled_links.push_back("r_palm");


//     for(std::vector<string>::iterator it = sampled_links.begin(); it != sampled_links.end(); it++)
//     {
//         control_points.insert(std::pair<string,Vector>(*it,_pRobot->GetLink(*it)->GetCOMOffset()));
//     }

//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_shin","r_shin"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_foot","r_foot"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_palm","l_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_index_prox","l_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_index_med","l_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("l_index_dist","l_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("r_palm","r_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("r_index_prox","r_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("r_index_med","r_thigh"));
//     self_collision_checking_pairs.push_back(std::pair<string,string>("r_index_dist","r_thigh"));
// }

void GeneralIK::GetRepulsiveVector(Vector& repulsive_vector, std::multimap<string,Vector>::iterator& control_point)
{
    dReal repulse_dist = 0.05;
    dReal repulse_constant = -1;
    Transform link_transform = _pRobot->GetLink(control_point->first)->GetTransform(); //need to verify if this is the actual transform of the link
    Vector control_point_global_position = link_transform*control_point->second;
    repulsive_vector = Vector(0,0,0);
    dReal shortest_dist = 100000000;

    //environment collision
    for(std::vector<KinBodyPtr>::iterator obs_it = _pObstacle.begin(); obs_it != _pObstacle.end(); obs_it++)
    {
        std::vector<KinBody::LinkPtr> ObstacleLink = (*obs_it)->GetLinks();
        for(std::vector<KinBody::LinkPtr>::iterator link_it = ObstacleLink.begin(); link_it != ObstacleLink.end(); link_it++)
        {
            // CollisionReportPtr report(new CollisionReport());
            // _pEnvironment->CheckCollision((*link_it),_pRobot->GetLink(control_point->first),report);
            // if(report->minDistance < repulse_dist)
            if(_pEnvironment->CheckCollision(_pRobot->GetLink(control_point->first),(*link_it)))
            {
                // shortest_dist = report->minDistance;
                // dReal largest_penetration_dist = -1000;
                RaveVector<dReal> repulsive_vector_component(0,0,0);
                // for(std::vector<CollisionReport::CONTACT>::iterator it = report->contacts.begin(); it != report->contacts.end(); it++)
                // {
                //     if(largest_penetration_dist < it->depth)
                //     {
                //         largest_penetration_dist = it->depth;
                //         repulsive_vector_component.x = it->norm.x;
                //         repulsive_vector_component.y = it->norm.y;
                //         repulsive_vector_component.z = it->norm.z;
                //     }
                // }
                repulsive_vector_component = obstacle_repulsive_vector[obs_it-_pObstacle.begin()];
                repulsive_vector = repulsive_vector + repulsive_vector_component;
            }
            // if(_pEnvironment->CheckCollision(_pRobot->GetLink(control_point->first),(*link_it)))
            // {
            //     std::vector<KinBody::Link::GeometryPtr> ObstacleGeometry = (*link_it)->GetGeometries();
            //     for(std::vector<KinBody::Link::GeometryPtr>::iterator geom_it = ObstacleGeometry.begin(); geom_it != ObstacleGeometry.end(); geom_it++)
            //     {
            //         GeometryType obstacle_geometry_type = (*geom_it)->GetType();
            //         RaveTransform<dReal> obstacle_geometry_transform = (*obs_it)->GetTransform() * (*link_it)->GetTransform() * (*geom_it)->GetTransform();
            //         RaveTransformMatrix<dReal> obstacle_rot_matrix = geometry::matrixFromQuat(obstacle_geometry_transform.rot);
            //         RaveTransformMatrix<dReal> inverse_obstacle_rot_matrix = obstacle_rot_matrix.inverse();
            //         RaveVector<dReal> obstacle_translation = obstacle_geometry_transform.trans;
            //         RaveVector<dReal> obstacle_frame_control_point_position = inverse_obstacle_rot_matrix * (control_point_global_position-obstacle_translation);
            //         dReal dist_to_obstacle = 0;
            //         RaveVector<dReal> repulsive_vector_component(0,0,0);
            //         RaveVector<dReal> nearest_point(0,0,0);
            //         if(obstacle_geometry_type == GT_Box)
            //         {
            //             Vector box_extents = (*geom_it)->GetBoxExtents();
            //             if(obstacle_frame_control_point_position.x > box_extents.x/2)
            //                 nearest_point.x = box_extents.x/2;
            //             else if(obstacle_frame_control_point_position.x < -box_extents.x/2)
            //                 nearest_point.x = -box_extents.x/2;
            //             else
            //                 nearest_point.x = obstacle_frame_control_point_position.x;
                        
            //             if(obstacle_frame_control_point_position.y > box_extents.y/2)
            //                 nearest_point.y = box_extents.y/2;
            //             else if(obstacle_frame_control_point_position.y < -box_extents.y/2)
            //                 nearest_point.y = -box_extents.y/2;
            //             else
            //                 nearest_point.y = obstacle_frame_control_point_position.y;

            //             if(obstacle_frame_control_point_position.z > box_extents.z/2)
            //                 nearest_point.z = box_extents.z/2;
            //             else if(obstacle_frame_control_point_position.z < -box_extents.z/2)
            //                 nearest_point.z = -box_extents.z/2;
            //             else
            //                 nearest_point.z = obstacle_frame_control_point_position.z;

            //             if(obstacle_frame_control_point_position.x != nearest_point.x ||
            //                obstacle_frame_control_point_position.y != nearest_point.y ||
            //                obstacle_frame_control_point_position.z != nearest_point.z)
            //             {
            //                 repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
            //                 dist_to_obstacle = repulsive_vector_component.lengthsqr3();
            //                 repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
            //             }
            //             else
            //             {
            //                 nearest_point = RaveVector<dReal>(0,0,0);
            //                 repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
            //                 repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
            //                 dist_to_obstacle = 0;
            //             }

            //         }
            //         else if(obstacle_geometry_type == GT_Sphere)
            //         {
            //             repulsive_vector_component = control_point_global_position - obstacle_translation;
            //             if(repulsive_vector_component.lengthsqr3() > (*geom_it)->GetSphereRadius())
            //                 dist_to_obstacle = repulsive_vector_component.lengthsqr3() - (*geom_it)->GetSphereRadius();
            //             else
            //                 dist_to_obstacle = 0;
            //             repulsive_vector_component = repulsive_vector_component.normalize3();
            //         }
            //         else if(obstacle_geometry_type == GT_Cylinder)
            //         {
            //             dReal cylinder_height = (*geom_it)->GetCylinderHeight();
            //             dReal cylinder_radius = (*geom_it)->GetCylinderRadius();
            //             //RaveVector<dReal> nearest_point(0,0,0);
            //             dReal xy_dist_to_centroid = sqrt(pow(obstacle_frame_control_point_position.x,2) + pow(obstacle_frame_control_point_position.y,2));

            //             if(xy_dist_to_centroid > cylinder_radius || fabs(obstacle_frame_control_point_position.z) > cylinder_height/2)
            //             {
            //                 nearest_point.x = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.x;
            //                 nearest_point.y = (cylinder_radius/xy_dist_to_centroid) * obstacle_frame_control_point_position.y;

            //                 if(obstacle_frame_control_point_position.z > cylinder_height/2)
            //                     nearest_point.z = cylinder_height/2;
            //                 else if(obstacle_frame_control_point_position.z < -cylinder_height/2)
            //                     nearest_point.z = -cylinder_height/2;
            //                 else
            //                     nearest_point.z = obstacle_frame_control_point_position.z;

            //                 repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
            //                 dist_to_obstacle = repulsive_vector_component.lengthsqr3();                    
            //             }
            //             else
            //             {
            //                 nearest_point = RaveVector<dReal>(0,0,0);
            //                 repulsive_vector_component = obstacle_frame_control_point_position - nearest_point;
            //                 dist_to_obstacle = 0;
            //             }

            //             repulsive_vector_component = (obstacle_rot_matrix*repulsive_vector_component).normalize3();
            //         }

            //         if(dist_to_obstacle < repulse_dist)
            //         {
            //             repulsive_vector = repulsive_vector + repulsive_vector_component;
            //         }

            //         if(dist_to_obstacle < shortest_dist)
            //             shortest_dist = dist_to_obstacle;


            //         // cout<<endl;
            //         // cout<<"Link in Collision: "<<control_point->first<<endl;
            //         // cout<<"Repulsive Vector: ("<<repulsive_vector_component.x<<","<<repulsive_vector_component.y<<","<<repulsive_vector_component.z<<")"<<endl;
            //         // cout<<"Control Point: ("<<control_point_global_position.x<<","<<control_point_global_position.y<<","<<control_point_global_position.z<<")"<<endl;
            //         // cout<<"Nearest Point: ("<<nearest_point.x<<","<<nearest_point.y<<","<<nearest_point.z<<")"<<endl;
            //         // cout<<"Obstacle Position: ("<<obstacle_translation.x<<","<<obstacle_translation.y<<","<<obstacle_translation.z<<")"<<endl;
            //         // string hhh;
            //         // cin >> hhh;
            //     }

            //     // repulsive_vector = Vector(0,0,1.0);

            // }
        }
    }

    //self collision
    for(std::vector<std::pair<string,string> >::iterator sc_it = self_collision_checking_pairs.begin(); sc_it != self_collision_checking_pairs.end(); sc_it++)
    {
        string link_1 = (*sc_it).first;
        string link_2 = (*sc_it).second;
        if(control_point->first == link_1 || control_point->first == link_2)
        {
            if(_pEnvironment->CheckCollision(_pRobot->GetLink(link_1),_pRobot->GetLink(link_2)))
            {
                RaveVector<dReal> repulsive_vector_component(0,0,0);
                string other_link = (control_point->first == link_1) ? link_2 : link_1;
                RaveVector<dReal> other_link_centroid = _pRobot->GetLink(other_link)->GetTransform().trans;
                repulsive_vector_component = (control_point_global_position - other_link_centroid).normalize3();
                repulsive_vector = repulsive_vector + repulsive_vector_component;
                shortest_dist = 0;
            }
        }
        
    }

    if(repulsive_vector.lengthsqr3() != 0)
    {
        repulsive_vector = repulse_constant * repulsive_vector * (1/repulsive_vector.lengthsqr3());
        // repulsive_vector = repulse_constant * exp(-shortest_dist) * repulsive_vector * (1/repulsive_vector.lengthsqr3());
        cout<<"Link: "<<control_point->first<<", Repulsive Vector: ("<<repulsive_vector.x<<","<<repulsive_vector.y<<","<<repulsive_vector.z<<")"<<endl;
        // cout<<"shortest_dist: "<<shortest_dist<<endl;
    }

    //for each obstacle, find its geometry type
    //calculate distance between the control point and the obstacle
    //calculate the repulsive from each obstacle according to ther relative position and distance
    //sum the repulsive vector
}

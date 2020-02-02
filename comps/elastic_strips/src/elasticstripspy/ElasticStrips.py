from __future__ import absolute_import, division, print_function

__author__ = 'yu-chi'

import openravepy as rave
import numpy as np
import IPython

from .TransformMatrix import SerializeTransform

# noinspection PyClassHasNoInit
class PlannerState:
    """
    Pseudo-enum to describe the PlannerState returned by ElasticStrips's GetPlannerState method
    """
    Idle = "idle"
    Planning = "planning"
    PlanSucceeded = "plansucceeded"
    PlanFailed = "planfailed"


class ElasticStrips(object):
    """
    Wrapper class for the OpenRave ElasticStrips plugin.

    :type env rave.Environment
    :type problem rave.Problem
    :type manip_indices dict[rave.Manipulator|str, int]
    """

    def __init__(self, env, robotname):
        """
        Create a ElasticStrips Problem Instance

        :param env: rave.Environment
        :param robotname: str
        :return: ElasticStrips
        """
        print('ElasticStrips.py initialize')

        self.module = rave.RaveCreateModule(env,'ElasticStrips')

        env.AddModule(self.module,'')
        self.env = env

        self.robotname = robotname
        # self.problem = rave.RaveCreateProblem(env, 'CBiRRT')
        # env.LoadProblem(self.problem, robotname)

        # Build a mapping for manipulators to manipulator indices
        self.manip_indices = {}
        for index, manip in enumerate(env.GetRobot(robotname).GetManipulators()):
            self.manip_indices[manip] = index
            self.manip_indices[manip.GetName()] = index

        # clone the robot for parallelization in C++
        # while(len(self.env.GetRobots()) < 5):
        #     new_robot = rave.RaveCreateRobot(self.env,'')
        #     new_robot.Clone(self.env.GetRobot(self.robotname),0)
        #     new_robot.SetName(self.robotname+"_clone_"+str(len(self.env.GetRobots())))
        #     self.env.AddRobot(new_robot)

    # def DoGeneralIK(self, execute=None, gettime=None, norot=None, returnclosest=None, robottm=None, maniptm=None,
    #                 checkcollisionlink=None, selfcollisionlinkpair=None, obstacles=None, movecog=None,
    #                 supportlinks=None, polyscale=None, polytrans=None, support=None, gravity=None, printcommand=False):
    #     """
    #     Get an IK solution with GeneralIK. In addition to these parameters, you can specify which DOF GeneralIK will
    #     use when planning by changing the active DOF

    #     :param bool execute: If True, GeneralIK will set the robot's configuration to the IK solution it found
    #                 (optional, defaults to False)
    #     :param bool gettime: If True, this function will return a 2-tuple of the result and the time in seconds that was
    #                 spent in the IK solver (optional, defaults to False)
    #     :param bool norot: If True, GeneralIK will ignore the rotation components of the goal transforms (optional,
    #                 defaults to False)
    #     :param bool returnclosest: If True, on failure GeneralIK will return the closest configuration it was able to
    #                 achieve. If this is True there is no feedback about whether the solver succeeded. (optional,
    #                 defaults to False)
    #     :param numpy.array robottm: Transform matrix representing the desired position and orientation of the base link
    #                 relative to the world frame (optional)
    #     :param list[(str|rave.Manipulator, numpy.array)] maniptm: List of tuples specifying a manipulator, by name
    #                 or Manipulator object, and its desired transform relative to the world frame. Any manipulator that
    #                 does not appear in this list but is in the active DOF list may be moved to avoid obstacles or keep
    #                 the robot balanced.
    #     :param list[float] movecog: The desired center of gravity of the robot as an [x, y, z] list. Support polygon
    #                 stability will not be performed if this parameter is not provided (optional)
    #     :param list[str] obstacles: The KinBody name that are taken as obstacle in the GeneralIK solver
    #     :param list[string|rave.KinBody.Link] supportlinks: Links to use to calculate the robot's support polygon,
    #                 specified either as the link name or link object. Setting this parameter enables balance checking
    #                 using the robot's support polygon. (optional)
    #     :param list[float] polyscale: [x, y, z] values that scale the support polygon. Z is ignored and may be ommitted.
    #                 (optional, defaults to [1, 1, 1])
    #     :param list[float] polytrans: [x, y, z] values that translate the support polygon. Z is ignored and may be
    #                 ommitted. (optional, defaults to [0, 0, 0])
    #     :param list[(str|rave.Manipulator, float)] support: Manipulators to use when calculating support using the
    #                 GIWC (Gravito-Inertial Wrench Cone). Each should be specified as a 2-tuple of the Manipulator object
    #                 or manipulator's name and the coefficient of friction to use for that manipulator. Specifying this
    #                 parameter enables GIWC support checking, and it should not be specified with the supportlinks
    #                 parameter.
    #     :param list[float] gravity: The [x, y, z] gravity vector to use when calculating support using GIWC (optional,
    #                 defaults to [0, 0, -9.8])
    #     :param bool printcommand: If this True, prints the resulting command string immediately before calling
    #                 GeneralIK, meant for debuggging (optional, defaults to False)
    #     :return list[float]|None|(list[float]|None, float): If gettime is False, the solution of the IK solver as a list
    #                 of joint angles, in the order specified by the active DOF list, or None if no solution was found.
    #                 If gettime is True, a 2-tuple of the solution or None and the time spent in the planner, measured
    #                 in seconds.
    #     """
    #     cmd = ["DoGeneralIK"]
    #     """:type : list[int | float | str]"""

    #     if execute is not None:
    #         cmd.append("exec")

    #     if gettime is not None:
    #         cmd.append("gettime")

    #     if norot is not None:
    #         cmd.append("norot")

    #     if returnclosest is not None:
    #         cmd.append("returnclosest")

    #     if robottm is not None:
    #         cmd.append("robottm")
    #         cmd.append(SerializeTransform(robottm))

    #     if maniptm is not None:
    #         cmd.append("nummanips")
    #         cmd.append(len(maniptm))

    #         for manip, tm in maniptm:
    #             cmd.append("maniptm")
    #             cmd.append(self.manip_indices[manip])
    #             cmd.append(SerializeTransform(tm))

    #     if checkcollisionlink is not None:
    #         cmd.append("checkcollision")
    #         cmd.append(len(checkcollisionlink))
    #         cmd.extend(rave_object_name(l) for l in checkcollisionlink)

    #     if selfcollisionlinkpair is not None:
    #         cmd.append("checkselfcollision")
    #         cmd.append(len(selfcollisionlinkpair))
    #         for l in selfcollisionlinkpair:
    #             cmd.extend([l[0],l[1]])

    #     if obstacles is not None:
    #         cmd.append("obstacles")
    #         cmd.append(len(obstacles))
    #         cmd.extend(rave_object_name(l) for l in obstacles)

    #     if movecog is not None:
    #         cmd.append("movecog")
    #         # Enumerating manually to ensure you can pass in any indexable object and get a movecog vector of 3 items
    #         cmd.append(movecog[0])
    #         cmd.append(movecog[1])
    #         cmd.append(movecog[2])

    #     if supportlinks is not None and support is not None:
    #         rave.raveLogWarn("Supportlinks and support should be mutually exclusive")

    #     if supportlinks is not None:
    #         cmd.append("supportlinks")
    #         cmd.append(len(supportlinks))
    #         cmd.extend(rave_object_name(l) for l in supportlinks)

    #     if polyscale is not None:
    #         if supportlinks is None:
    #             rave.raveLogWarn("Polyscale has no effect when supportlinks is not provided")
    #         cmd.append("polyscale")
    #         cmd.extend(polyscale)
    #         if len(polyscale) == 2:
    #             cmd.append(0)
    #         elif len(polyscale) != 3:
    #             raise ValueError("Polyscale must have either 2 or 3 values")

    #     if polytrans is not None:
    #         if supportlinks is None:
    #             rave.raveLogWarn("Polytrans has no effect when supportlinks is not provided")
    #         cmd.append("polytrans")
    #         cmd.extend(polytrans)
    #         if len(polytrans) == 2:
    #             cmd.append(0)
    #         elif len(polytrans) != 3:
    #             raise ValueError("Polytrans must have either 2 or 3 values")

    #     if support is not None:
    #         for manip, mu in support:
    #             cmd.append("support {} {}".format(rave_object_name(manip), mu))

    #     if gravity is not None:
    #         cmd.append("gravity")
    #         # Enumerating manually to ensure you can pass in any indexable object and get a gravity vector of 3 items
    #         cmd.append(gravity[0])
    #         cmd.append(gravity[1])
    #         cmd.append(gravity[2])

    #     cmd_str = " ".join(str(item) for item in cmd)

    #     if printcommand:
    #         print(cmd_str)

    #     result_str = self.problem.SendCommand(cmd_str)
    #     result = [float(x) for x in result_str.split()]

    #     if gettime:
    #         # Convert the time from ms to seconds to match what is expected in Python
    #         time = result.pop() / 1000.0

    #         if not result:
    #             return time, None
    #         else:
    #             return time, result
    #     else:
    #         if not result:
    #             return None
    #         else:
    #             return result


    def RunElasticStrips(self, manips=None, trajectory=None, gettime=None, contact_manips=None, desiredmanippose=None, checkcollisionlink=None, selfcollisionlinkpair=None, obstacles=None, posturecontrol=None,
                    supportlinks=None, polyscale=None, polytrans=None, support=None, gravity=None, printcommand=False):
        """
        Get an IK solution with GeneralIK. In addition to these parameters, you can specify which DOF GeneralIK will
        use when planning by changing the active DOF

        :param bool execute: If True, GeneralIK will set the robot's configuration to the IK solution it found
                    (optional, defaults to False)
        :param bool gettime: If True, this function will return a 2-tuple of the result and the time in seconds that was
                    spent in the IK solver (optional, defaults to False)
        :param bool norot: If True, GeneralIK will ignore the rotation components of the goal transforms (optional,
                    defaults to False)
        :param bool returnclosest: If True, on failure GeneralIK will return the closest configuration it was able to
                    achieve. If this is True there is no feedback about whether the solver succeeded. (optional,
                    defaults to False)
        :param numpy.array robottm: Transform matrix representing the desired position and orientation of the base link
                    relative to the world frame (optional)
        :param list[(str|rave.Manipulator, numpy.array)] maniptm: List of tuples specifying a manipulator, by name
                    or Manipulator object, and its desired transform relative to the world frame. Any manipulator that
                    does not appear in this list but is in the active DOF list may be moved to avoid obstacles or keep
                    the robot balanced.
        :param list[float] movecog: The desired center of gravity of the robot as an [x, y, z] list. Support polygon
                    stability will not be performed if this parameter is not provided (optional)
        :param list[str] obstacles: The KinBody name that are taken as obstacle in the GeneralIK solver
        :param list[string|rave.KinBody.Link] supportlinks: Links to use to calculate the robot's support polygon,
                    specified either as the link name or link object. Setting this parameter enables balance checking
                    using the robot's support polygon. (optional)
        :param list[float] polyscale: [x, y, z] values that scale the support polygon. Z is ignored and may be ommitted.
                    (optional, defaults to [1, 1, 1])
        :param list[float] polytrans: [x, y, z] values that translate the support polygon. Z is ignored and may be
                    ommitted. (optional, defaults to [0, 0, 0])
        :param list[(str|rave.Manipulator, float)] support: Manipulators to use when calculating support using the
                    GIWC (Gravito-Inertial Wrench Cone). Each should be specified as a 2-tuple of the Manipulator object
                    or manipulator's name and the coefficient of friction to use for that manipulator. Specifying this
                    parameter enables GIWC support checking, and it should not be specified with the supportlinks
                    parameter.
        :param list[float] gravity: The [x, y, z] gravity vector to use when calculating support using GIWC (optional,
                    defaults to [0, 0, -9.8])
        :param bool printcommand: If this True, prints the resulting command string immediately before calling
                    GeneralIK, meant for debuggging (optional, defaults to False)
        :return list[float]|None|(list[float]|None, float): If gettime is False, the solution of the IK solver as a list
                    of joint angles, in the order specified by the active DOF list, or None if no solution was found.
                    If gettime is True, a 2-tuple of the solution or None and the time spent in the planner, measured
                    in seconds.
        """

        cmd = ["RunElasticStrips"]
        """:type : list[int | float | str]"""

        cmd.append(self.robotname)

        if manips is not None:
            cmd.append("nummanips")
            cmd.append(len(manips))
            cmd.append("manips")
            for m in manips:
                cmd.append(self.manip_indices[m])

        if trajectory is not None:
            cmd.append("trajectory")
            cmd.append(len(trajectory))
            for waypoint in trajectory:
                for dim in waypoint:
                    cmd.append(dim) # dim includes the index and joint angle value

        if gettime is not None:
            cmd.append("gettime")

        if contact_manips is not None:
            cmd.append("contact_manips")
            cmd.append(len(contact_manips))
            for i,manips in enumerate(contact_manips):
                cmd.append(i)
                cmd.append(len(manips))
                for manip in manips:
                    cmd.append(manip[0])
                    cmd.append(manip[1])

        if desiredmanippose is not None:
            cmd.append("desiredmanippose")
            cmd.append(len(desiredmanippose))
            for waypoints in desiredmanippose:
                cmd.append(waypoints[0][0]) # index
                cmd.append(len(waypoints))
                for wp in waypoints:
                    cmd.append(wp[1])
                    cmd.append(wp[2])
                    cmd.append(SerializeTransform(wp[3]))

        if checkcollisionlink is not None:
            cmd.append("checkcollision")
            cmd.append(len(checkcollisionlink))
            cmd.extend(rave_object_name(l) for l in checkcollisionlink)

        if selfcollisionlinkpair is not None:
            cmd.append("checkselfcollision")
            cmd.append(len(selfcollisionlinkpair))
            for l in selfcollisionlinkpair:
                cmd.extend([l[0],l[1]])

        if obstacles is not None:
            cmd.append("obstacles")
            cmd.append(len(obstacles))
            for obs in obstacles:
                cmd.append(rave_object_name(obs[0]))
                cmd.append(obs[1][0])
                cmd.append(obs[1][1])
                cmd.append(obs[1][2])
            
            # cmd.extend(rave_object_name(l) for l in obstacles)

        if posturecontrol is not None:
            cmd.append("posturecontrol")
            cmd.append(len(posturecontrol))
            for link,tm in posturecontrol:
                    cmd.append(link)
                    cmd.append(SerializeTransform(tm))

        if supportlinks is not None and support is not None:
            rave.raveLogWarn("Supportlinks and support should be mutually exclusive")

        if supportlinks is not None:
            cmd.append("supportlinks")
            cmd.append(len(supportlinks))
            cmd.extend(rave_object_name(l) for l in supportlinks)

        if polyscale is not None:
            if supportlinks is None:
                rave.raveLogWarn("Polyscale has no effect when supportlinks is not provided")
            cmd.append("polyscale")
            cmd.extend(polyscale)
            if len(polyscale) == 2:
                cmd.append(0)
            elif len(polyscale) != 3:
                raise ValueError("Polyscale must have either 2 or 3 values")

        if polytrans is not None:
            if supportlinks is None:
                rave.raveLogWarn("Polytrans has no effect when supportlinks is not provided")
            cmd.append("polytrans")
            cmd.extend(polytrans)
            if len(polytrans) == 2:
                cmd.append(0)
            elif len(polytrans) != 3:
                raise ValueError("Polytrans must have either 2 or 3 values")

        if support is not None:
            for manip, mu in support:
                cmd.append("support {} {}".format(rave_object_name(manip), mu))

        if gravity is not None:
            cmd.append("gravity")
            # Enumerating manually to ensure you can pass in any indexable object and get a gravity vector of 3 items
            cmd.append(gravity[0])
            cmd.append(gravity[1])
            cmd.append(gravity[2])

        cmd_str = " ".join(str(item) for item in cmd)

        if printcommand:
            print(cmd_str)

        result_str = self.module.SendCommand(cmd_str,True)
        result = [float(x) for x in result_str.split()]

        if gettime:
            # Convert the time from ms to seconds to match what is expected in Python
            time = result.pop() / 1000.0

            if not result:
                return time, None
            else:
                return time, result
        else:
            if not result:
                return None
            else:
                return result


def rave_object_name(obj):
    """
    Accepts an openrave object or its name and returns its name
    :param obj: The object whose name to return
    :return str: The object's name
    """
    return obj if isinstance(obj, str) else obj.GetName()


def load_traj_from_file(traj_path, env):
    """
    Parses a trajectory file at the given path and returns the trajectory object.
    Source: https://personalrobotics.ri.cmu.edu/intel-pkg/libherb/html/traj__util_8py_source.html

    :param str traj_path: The file path
    :param rave.Environment env: The OpenRAVE environment
    :return rave.Trajectory: The trajectory object
    """
    try:
        with open(traj_path) as f:
            traj_str = f.read()
        traj = rave.RaveCreateTrajectory(env, '').deserialize(traj_str)
        if traj is None:
            raise Exception('traj is None after parsing traj_str=\'%s\'' % traj_str)
        return traj
    except Exception as e:
        raise Exception('Could not load trajectory from \'%s\': %s' % (traj_path, str(e)))

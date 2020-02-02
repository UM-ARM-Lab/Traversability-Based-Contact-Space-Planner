from __future__ import absolute_import, division, print_function

__author__ = 'will'

import openravepy as rave
import numpy as np

from .TSR import TSRChain
from .TransformMatrix import SerializeTransform


# noinspection PyClassHasNoInit
class PlannerState:
    """
    Pseudo-enum to describe the PlannerState returned by CBiRRT's GetPlannerState method
    """
    Idle = "idle"
    Planning = "planning"
    PlanSucceeded = "plansucceeded"
    PlanFailed = "planfailed"


class CBiRRT(object):
    """
    Wrapper class for the OpenRave CBiRRT plugin.

    :type env rave.Environment
    :type problem rave.Problem
    :type manip_indices dict[rave.Manipulator|str, int]
    """
    START = 0b100
    PATH = 0b010
    GOAL = 0b001

    def __init__(self, env, robotname):
        """
        Create a CBiRRT Problem Instance

        :param env: rave.Environment
        :param robotname: str
        :return: CBiRRT
        """
        self.env = env
        self.problem = rave.RaveCreateProblem(env, 'CBiRRT')
        env.LoadProblem(self.problem, robotname)

        # Build a mapping for manipulators to manipulator indices
        self.manip_indices = {}
        for index, manip in enumerate(env.GetRobot(robotname).GetManipulators()):
            self.manip_indices[manip] = index
            self.manip_indices[manip.GetName()] = index

    def GrabBody(self, body):
        """
        Call GrabBody, a CBiRRT command that calls robot->grab with the given KinBody and has no return value

        :param rave.KinBody body: The KinBody to grab
        """
        self.problem.SendCommand('GrabBody name ' + body.GetName())

    def RunCBiRRT(self, jointgoals=None, jointstarts=None, ikguess=None, Tattachedik_0=None, smoothingitrs=None,
                  filename=None, timelimit=None, TSRChains=None, supportlinks=None, polyscale=None, polytrans=None,
                  support=None, gravity=None, heldobject=None, psample=None, bikfastsinglesolution=None,
                  planinnewthread=None, allowlimadj=None, smoothtrajonly=None, outputtrajobj=False, printcommand=False):
        """
        Run the CBiRRT planner.

        This command has many parameters which are fully documented below, but here is an overview:

        The common use case, planning a path with TSR constraints, usually involves all or most of these parameters:
        -- TSRChains, which describe the end effector constraints,
        -- psample, which is required in order to sample from goal TSRs
        -- outputtrajobj, which returs the planned trajectory
        -- supportlinks or support, which do stability checking
        -- timelimit, which limits the time taken by CBiRRT

        These parameters change certian aspects of CBiRRT's behavior:
        -- jointgoals, which sets the goal pose or poses (unclear whether that is instead of or in addition to sampling)
        -- jointstarts, which sets the start pose or poses (default behavior is to use the current pose)
        -- ikguess, which proivdes an initial guess for the iterative IK solver when creating the initial goal pose.
                    Setting ikguess can sometimes help if CBiRRT always returns "Unable to find a goal IK solution".
        -- smoothingitrs, which controls how much smoothing will be performed. Increasing this value will usually
                    reduce cases where the robot makes unnecessarily complex motions.
        -- bikfastsinglesolution, which is described below

        Some sets of parameters are related or mutually exclusive:
        -- polyscale and polytrans only have effect if you specify supportlinks
        -- gravity only has effect if you specify support
        -- supportlinks and support are mutually exclusive (they use different support modes)
        -- planinnewthread, smoothtrajonly, and outputtrajobj are all mutually exclusive
        -- smoothtrajonly causes every parameter except smoothingitrs and parameters which apply constraints on the path
                    to be ignored

        :param list[float]|list[list[float]] jointgoals: Desired joint value vector at end of planning or list of
                    desired joint value vectors (optional)
        :param list[float]|list[list[float]] jointstarts: Joint value vector at start of planning or list of desired
                    joint value vectors (optional, defaults to current
                    configuration)
        :param list[float] ikguess: Joint value vector that the itertive IK solver will use as an initial guess when
                    sampling goals (optional)
        :param dict[int, np.array|np.matrix] Tattachedik_0: Transformation matrix between attachedik frame and 0 frame
                    for each manipulator (optional, defaults to identity)
        :param int smoothingitrs: Number of iterations of smoothing to apply to the final trajectory (optional,
                    defaults to 300)
        :param str filename: Filename of generated trajectory file (optional, defaults to "cmovetraj.txt")
        :param float timelimit: Time limit in seconds before planning gives up (optional, defaults to 25 seconds)
        :param list[TSRChain] TSRChains: A list of TSR chains to constrain the path, start, and/or goal (optional)
        :param list[string|rave.KinBody.Link] supportlinks: Links to use to calculate the robot's support polygon,
                    specified either as the link name or link object. Setting this parameter enables balance checking
                    using the robot's support polygon. (optional)
        :param list[float] polyscale: [x, y, z] values that scale the support polygon. Z is ignored and may be ommitted.
                    (optional, defaults to [1, 1, 1])
        :param list[float] polytrans: [x, y, z] values that translate the support polygon. Z is ignored and may be
                    ommitted. (optional, defaults to [0, 0, 0])
        :param list[(str|rave.Manipulator, float, int)] support: Manipulators to use when calculating support using the
                    GIWC (Gravito-Inertial Wrench Cone). Each should be specified as a 3-tuple of the Manipulator object
                    or manipulator's name, the coefficient of friction to use for that manipulator, and a bitmask
                    indicating whether the manipulator is supporting the robot at the start, path, and/or goal. The
                    CBiRRT.START, CBiRRT.PATH, and CBiRRT.GOAL variables can be used with boolean or to construct this
                    bitmask. Note that path constraints also apply at the start and goal, so if the goal support is a
                    superset of the path support it has no effect. Specifying this parameter enables GIWC support
                    checking, and it should not be specified with the supportlinks parameter.
        :param list[float] gravity: The [x, y, z] gravity vector to use when calculating support using GIWC (optional,
                    defaults to [0, 0, -9.8])
        :param str|rave.KinBody heldobject: A KinBody that robot is holding, specified as the KinBody object or name.
                    This body will be considered when calculating whether the robot is balanced. Only one is supported.
                    (optional)
        :param float psample: Probability of sampling from the TSR chains rather than growing the RRT trees on each
                    iteration. Defaults to 0, which is usually not desired behavior. This module will output a warning
                    if psample is not specified and TSRs are provided. To silence this warning, specifically set psample
                    to 0. (optional, defaults to 0)
        :param boolean bikfastsinglesolution: Controls the use of attached IK solvers (such as ikfast) when sampling
                    start and goal configurations. If this is true, CBiRRT will get a single IK solution with collision
                    checking enabled in the IK solver. If fasle, it will get multiple IK solutoins with collision
                    checking disabled, and perform collision checking on them later. (optional, defaults to True)
        :param boolean planinnewthread: Whether to run the planner in a new thread. If this is true then RunCBiRRT
                    returns quickly and GetPlannerState and StopPlanner must be used to interact with the planner.
                    (optional, defaults to False)
        :param boolean allowlimadj: Allow CBiRRT to temporarily adjust joint limits up or down so that the start
                    configuration is always within joint limits (optional, defaults to False)
        :param str smoothtrajonly: If this parameter is provided, CBiRRT just performs trajectory smoothing on the
                    trajectory file. The value of this argument is the filename of the trajectory to smooth (optional)
        :param bool outputtrajobj: If outputtrajobj is True then this method returns a 2-tuple of CBiRRT's output and
                    the resulting trajectory. This is incompatible with the planinnewthread parameter. (optional,
                    defaults to False)
        :param bool printcommand: If this True, prints the resulting command string immediately before calling CBiRRT,
                    meant for debuggging (optional, defaults to False)
        :return str|(str, rave.Trajectory): If outputtrajobj is False, returns the output of CBiRRT. Otheriwse
                    returns a tuple of the output and the decoded trajectory.
        """
        cmd = ["RunCBiRRT"]
        """:type : list[int | float | str]"""

        if jointgoals is not None:
            try:  # Assume it's a list of lists of goals
                for jointgoal in jointgoals:
                    cmd.append("jointgoals")
                    cmd.append(len(jointgoal))
                    cmd.extend(jointgoal)
            except TypeError:  # Fall back to single list of goals
                # "jointgoals" will have already been appended
                cmd.append(len(jointgoals))
                cmd.extend(jointgoals)

        if jointstarts is not None:
            try:  # Assume it's a list of lists of starts
                for jointstart in jointstarts:
                    cmd.append("jointstarts")
                    cmd.append(len(jointstart))
                    cmd.extend(jointstart)
            except TypeError:  # Fall back to single list of starts
                # "jointstarts" will have already been appended
                cmd.append(len(jointstarts))
                cmd.extend(jointstarts)

        if ikguess is not None:
            cmd.append("ikguess")
            cmd.append(len(ikguess))
            cmd.extend(ikguess)

        if Tattachedik_0 is not None:
            for manipindex, T in Tattachedik_0.items():
                cmd.append("Tattachedik_0")
                cmd.append(manipindex)
                cmd.append(SerializeTransform(T))

        if smoothingitrs is not None:
            cmd.append("smoothingitrs")
            cmd.append(smoothingitrs)

        if filename is not None:
            cmd.append("filename")
            cmd.append(filename)

        if timelimit is not None:
            cmd.append("timelimit")
            cmd.append(timelimit)

        if TSRChains is not None:
            for chain in TSRChains:
                # chain.serialize appends the "TSRChain" label as well
                cmd.append(chain.serialize())

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
            for manip, mu, mode in support:
                cmd.append("support {} {} {:b}".format(rave_object_name(manip), mu, mode))

        if gravity is not None:
            cmd.append("gravity")
            # Enumerating manually to ensure you can pass in any indexable object and get a gravity vector of 3 items
            cmd.append(gravity[0])
            cmd.append(gravity[1])
            cmd.append(gravity[2])

        if heldobject is not None:
            cmd.append("heldobject")
            cmd.append(heldobject)

        if psample is not None:
            cmd.append("psample")
            cmd.append(psample)

        if bikfastsinglesolution is not None:
            cmd.append("bikfastsinglesolution")
            cmd.append(int(bikfastsinglesolution))

        if planinnewthread is not None:
            cmd.append("planinnewthread")
            cmd.append(int(planinnewthread))

        if allowlimadj is not None:
            cmd.append("allowlimadj")
            cmd.append(int(allowlimadj))

        if smoothtrajonly is not None:
            # This is a confusing parameter name, so add an assert to try to make sure it's being used correctly
            if isinstance(smoothtrajonly, bool) or isinstance(smoothtrajonly, int):
                rave.raveLogWarn("smoothtrajonly should be a filename, not a boolean")
            cmd.append("smoothtrajonly")
            cmd.append(smoothtrajonly)

        cmd_str = " ".join(str(item) for item in cmd)

        if printcommand:
            print(cmd_str)

        cbirrt_result = self.problem.SendCommand(cmd_str)

        if cbirrt_result is None:
            raise rave.PlanningError("CBiRRT Unknown Error")
        elif cbirrt_result[0] == '0':
            raise rave.PlanningError(cbirrt_result.lstrip("0 "))

        if outputtrajobj:
            trajobj = load_traj_from_file(filename or "cmovetraj.txt", self.env)
            return (cbirrt_result.lstrip("1 "), trajobj)
        else:
            return cbirrt_result.lstrip("1 ")


    def DoGeneralIK(self, execute=None, gettime=None, norot=None, returnclosest=None, robottm=None, maniptm=None,
                    checkcollisionlink=None, selfcollisionlinkpair=None, obstacles=None, movecog=None,
                    supportlinks=None, polyscale=None, polytrans=None, support=None, gravity=None, reusegiwc=None, exact_com=None, printcommand=False):
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
        :param list[(str,(float,float,float))] obstacles: The KinBody name that are taken as obstacle in the GeneralIK solver, and repulsive vector direction.(if specified)
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
        cmd = ["DoGeneralIK"]
        """:type : list[int | float | str]"""

        if execute is True:
            cmd.append("exec")

        if gettime is True:
            cmd.append("gettime")

        if norot is True:
            cmd.append("norot")

        if returnclosest is True:
            cmd.append("returnclosest")

        if robottm is not None:
            cmd.append("robottm")
            cmd.append(SerializeTransform(robottm))

        if maniptm is not None:
            cmd.append("nummanips")
            cmd.append(len(maniptm))

            for manip, tm in maniptm:
                cmd.append("maniptm")
                cmd.append(self.manip_indices[manip])
                cmd.append(SerializeTransform(tm))

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
                cmd.append(obs[0])
                if(obs[1] is None):
                    cmd.extend([0,0,0])
                else:
                    cmd.append(obs[1][0])
                    cmd.append(obs[1][1])
                    cmd.append(obs[1][2])

        if movecog is not None:
            cmd.append("movecog")
            # Enumerating manually to ensure you can pass in any indexable object and get a movecog vector of 3 items
            cmd.append(movecog[0])
            cmd.append(movecog[1])
            cmd.append(movecog[2])

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

        if reusegiwc is True:
            cmd.append("reusegiwc")

        if exact_com is True:
            cmd.append("exactcom")

        cmd_str = " ".join(str(item) for item in cmd)

        if printcommand:
            print(cmd_str)

        result_str = self.problem.SendCommand(cmd_str,True) # releasegil = True
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


    def RunElasticStrips(self, manips=None, trajectory=None, gettime=None, desiredmanippose=None, contact_manips=None, checkcollisionlink=None, selfcollisionlinkpair=None, obstacles=None, posturecontrol=None, movecog=None,
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
        :param list[(str,(float,float,float))] obstacles: The KinBody name that are taken as obstacle in the GeneralIK solver, and repulsive vector direction.(if specified)
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

        if manips is not None:
            cmd.append("nummanips")
            cmd.append(len(manips))

            for m in manips:
                cmd.append(self.manip_indices[m])

        if trajectory is not None:
            cmd.append("trajectory")
            cmd.append(len(trajectory))
            for index, tm in trajectory:
                cmd.append(index)
                cmd.append(SerializeTransform(tm))

        if gettime is not None:
            cmd.append("gettime")

        if contact_manips is not None:
            for i,manips in enumerate(contact_manips):
                cmd.append("contact_manips")
                cmd.append(i)
                cmd.append(len(manips))
                for manip in manips:
                    cmd.append(manip)

        if desiredmanippose is not None:
            for waypoints in desiredmanippose:
                cmd.append("desiredmanippose")
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
            cmd.extend(rave_object_name(l) for l in obstacles)

        # if obstacles is not None:
        #     cmd.append("obstacles")
        #     cmd.append(len(obstacles))
        #     for obs in obstacles:
        #         cmd.append(obs[0])
        #         if(obs[1] is None):
        #             cmd.extend([0,0,0])
        #         else:
        #             cmd.append(obs[1][0])
        #             cmd.append(obs[1][1])
        #             cmd.append(obs[1][2])

        if posturecontrol is not None:
            cmd.append("posturecontrol")
            cmd.append(len(posturecontrol))
            for link,tm in posturecontrol:
                    cmd.append(link)
                    cmd.append(SerializeTransform(tm))

        if movecog is not None:
            cmd.append("movecog")
            # Enumerating manually to ensure you can pass in any indexable object and get a movecog vector of 3 items
            cmd.append(movecog[0])
            cmd.append(movecog[1])
            cmd.append(movecog[2])

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

        result_str = self.problem.SendCommand(cmd_str)
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

    def CheckSupport(self, supportlinks=None, heldobject=None, polyscale=None, polytrans=None, draw=False,
                     printcommand=False):
        """
        Check robot support according to polygon-based stability. This function does not currently support GIWC
        stability checking.

        :param list[string|rave.KinBody.Link] supportlinks: Links to use to calculate the robot's support polygon,
                    specified either as the link name or link object. Setting this parameter enables balance checking
                    using the robot's support polygon. (optional)
        :param str|rave.KinBody heldobject: A KinBody that robot is holding, specified as the KinBody object or name.
                    This body will be considered when calculating whether the robot is balanced. Only one is supported.
                    (optional)
        :param list[float] polyscale: [x, y, z] values that scale the support polygon. Z is ignored and may be ommitted.
                    (optional, defaults to [1, 1, 1])
        :param list[float] polytrans: [x, y, z] values that translate the support polygon. Z is ignored and may be
                    ommitted. (optional, defaults to [0, 0, 0])
        :param bool draw: If True, the support polygon and robot's center of mass will be drawn in the viewer (optional,
                    defaults to False)
        :param bool printcommand: If this True, prints the resulting command string immediately before calling CBiRRT,
                    meant for debuggging (optional, defaults to False)
        :return bool: True if the robot is supported, false otherwise
        """
        cmd = ["CheckSupport"]
        """:type : list[int | float | str]"""

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

        if heldobject is not None:
            cmd.append("heldobject")
            cmd.append(heldobject)

        if draw is not None:
            cmd.append("draw")

        cmd_str = " ".join(str(item) for item in cmd)

        if printcommand:
            print(cmd_str)

        result = self.problem.SendCommand(cmd_str)
        return bool(int(result))

    def CheckSelfCollision(self):
        """
        Runs robot->CheckSelfCollision

        :return bool: True if the robot is in self-collision, False otherwise
        """
        return bool(int(self.problem.SendCommand("CheckSelfCollision")))

    def GetJointAxis(self, obj, jointind):
        """
        Returns the joint axis for the specified joint of the specified KinBody

        :param openravepy.KinBody|str obj: The KinBody object or name
        :param int jointind: The joint index
        :return list[float]: The joint axis
        """
        result = self.problem.SendCommand("GetJointAxis {} {}".format(rave_object_name(obj), jointind))
        return [float(x) for x in result.split()]

    def GetJointTransform(self, obj, jointind):
        """
        Returns the joint transform for the specified joint of the specified KinBody

        :param openravepy.KinBody|str obj: The KinBody object or name
        :param int jointind: The joint index
        :return list[int]: The joint transform
        """
        result = self.problem.SendCommand("GetJointAxis {} {}".format(rave_object_name(obj), jointind))
        result = np.array([float(x) for x in result.split()])
        return np.append(result.reshape((4, 3)).T, [[0, 0, 0, 1]], axis=0)

    def GetCamView(self):
        """
        Returns the current camera pose as a 7-element list of position and quaternion

        :return list[float]: The camera pose
        """
        return [float(x) for x in self.problem.SendCommand("GetCameraPose").split()]

    def SetCamView(self, pose):
        """
        Sets the camera pose to a 7-element list of position and quaternion

        :param list[float] pose: The camera pose
        """
        self.problem.SendCommand("SetCameraPose " + " ".join(str(x) for x in pose))

    def Traj(self, filename):
        """
        Execute a trajectory from the given file

        :param str filename: The filename of the trajectory to execute
        :return bool: Boolean indicatings success, appears to always return True
        """
        return bool(int(self.problem.SendCommand("Traj " + filename)))

    def GetPlannerState(self):
        """
        Returns the planner's current state. Comparisons involving the returned state can use PlannerState.Idle,
        PlannerState.Planning, PlannerState.PlanSucceeded, and PlannerState.PlanFailed.

        :return str: One of PlannerState.Idle, PlannerState.Planning, PlannerState.PlanSucceeded, or
                    PlannerState.PlanFailed
        """
        return self.problem.SendCommand("GetPlannerState")

    def StopPlanner(self):
        """
        Stops the planner and returns a human-friendly success or error message. Stopping the planner sets its state
        to PlannerState.PlanFailed.

        :return str: The success or error message
        """
        return self.problem.SendCommand("StopPlanner")

    def ClearDrawn(self):
        """
        Removes all objects drawn in the visualizer by any command in CBiRRT.
        """
        self.problem.SendCommand("ClearDrawn")


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

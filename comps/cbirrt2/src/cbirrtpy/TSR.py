# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
from __future__ import print_function

"""
Classes for representing and serializing TSRs and TSR Chains
"""

import numpy as np
from TransformMatrix import SerializeTransform, Serialize1DIntegerMatrix, Serialize1DMatrix
import copy

class TSR(object):
    def __init__(self, manipindex, T0_w=None, Tw_e=None, Bw=None, bodyandlink="NULL"):
        """
        Create a TSR
        :param int manipindex: the 0-indexed index of the robot's manipulator
        :param np.matrix T0_w: transform matrix of the TSR's reference frame relative to the 0 frame
        :param np.matrix Tw_e: transform matrix of the TSR's offset frame relative the w frame
        :param np.matrix Bw: column vector of [lower, upper] bound pairs in Tx, Ty, Tz, Rx, Ry, Rz.
        :param str bodyandlink: body and link which is used as the 0 frame. Format 'body_name link_name'. To use the
                world frame, specify 'NULL' or leave this argument blank
        :return:
        """
        self.T0_w = T0_w if T0_w is not None else np.identity(4)
        self.Tw_e = Tw_e if Tw_e is not None else np.identity(4)
        self.Bw = Bw if Bw is not None else np.mat(np.zeros( (1, 12) ))
        self.manipindex = manipindex
        self.bodyandlink = bodyandlink

    def serialize(self):
        """
        Serialize this TSR in the format used by CoMPS (the Constrained Motion Planning Suite)
        :return str: The serialized TSR
        """
        return '%d %s %s %s %s' % (self.manipindex, self.bodyandlink, SerializeTransform(self.T0_w),
                                   SerializeTransform(self.Tw_e), Serialize1DMatrix(self.Bw))

    def draw_pos(self, env, color=(0.25, 0.5, 0.75, 0.35)):
        """
        Draw the position component of simple TSRs. It does not support displaying the rotation component, and it will
        not work for any TSR which is curved. This function is still under development -- currently, point and line
        TSRs are supported, plane and box TSRs are not.

        :param openravepy.Environment env: The OpenRAVE environment to draw in
        :param (float, float, float, float) color: The (r, g, b, a) color to use to draw the TSR
        :return openravepy.GraphHandle: The graph handle of the drawn TSR. This may later change to be a list of
                    graph handles.
        """

        # Find the number of dimensions where the lower and upper bounds are equal, which determines whether the TSR
        # is a point TSR, line TSR, plane TSR or box TSR
        zero_dims = sum(1 for i in range(0, 6, 2) if self.Bw[0,i] == self.Bw[0, i+1])

        tsr_pos_origin = np.dot(self.T0_w, np.array([0,0,0,1]))
        pos_lower = np.append(self.Bw[0, 0:6:2], np.mat([1]), axis=1)
        pos_upper = np.append(self.Bw[0, 1:6:2], np.mat([1]), axis=1)

        if zero_dims == 3: # Point TSR
            point = np.dot(self.T0_w, tsr_pos_origin)
            return env.plot3(np.array(point[0:3]), 5, color)
        elif zero_dims == 2: # Line TSR
            end1 = np.dot(self.T0_w, pos_lower.T)
            end2 = np.dot(self.T0_w, pos_upper.T)
            return env.drawlinelist(np.array(np.vstack((end1.T[0, 0:3], end2.T[0, 0:3]))), 5, color)
        elif zero_dims == 1: # Plane TSR
            # aabb_center = np.mean(np.vstack((pos_lower, pos_upper)), axis=0)
            # center = np.dot(self.T0_w, aabb_center)
            # print(center)
            # aabb_extents = aabb_center - pos_lower
            #
            # return env.drawplane()
            pass


class TSRChain(object):
    def __init__(self, TSRs=None, SampleStart=False, SampleGoal=False, Constrain=False, mimicbodyname="NULL",
                 mimicbodyjoints=None):
        """
        Create a TSRChain
        :param list[TSR] TSRs: The TSRs to insert, if any. TSRs can be added later using insertTSR()
        :param boolean SampleStart: Whether to sample start states from this TSRChain
        :param boolean SampleGoal: Whether to sample goal states from this TSRChain
        :param boolean Constrain: Whether to constrain the entire path to this TSRChain
        :param str mimicbodyname: TODO: Document
        :param list[int] mimicbodyjoints: TODO: Document
        :return TSRChain:
        """
        self.TSRs = []
        self.bSampleStartFromChain = int(SampleStart)
        self.bSampleGoalFromChain = int(SampleGoal)
        self.bConstrainToChain = int(Constrain)
        self.mimicbodyname = mimicbodyname
        self.mimicbodyjoints = mimicbodyjoints if mimicbodyjoints is not None else []

        if TSRs is not None:
            for tsr in TSRs:
                self.insertTSR(tsr)

    def insertTSR(self, tsr_in):
        """
        Add a TSR to this TSRChain

        :param TSR tsr_in: The TSR to add
        """
        self.TSRs.append(copy.deepcopy(tsr_in))

    def serialize(self):
        """
        Serialize this TSRChain into a string

        :return string: The serialized TSRChain in the format accepted by CoMPS
        """
        allTSRstring = ' '.join(tsr.serialize() for tsr in self.TSRs)
        numTSRs = len(self.TSRs)
        outstring = 'TSRChain %d %d %d %d %s %s' % (self.bSampleStartFromChain, self.bSampleGoalFromChain,
                                                    self.bConstrainToChain, numTSRs, allTSRstring, self.mimicbodyname)
        if np.size(self.mimicbodyjoints) != 0:
            outstring += ' %d %s ' % (np.size(self.mimicbodyjoints), Serialize1DIntegerMatrix(self.mimicbodyjoints))
  
        return outstring

    def setFirstT0_w(self,T0_w_in):
        """
        Override the T0_w of the first TSR in this chain. Assumes there is at least one TSR.
        :param np.matrix T0_w_in: The new T0_w
        """
        self.TSRs[0].T0_w = copy.deepcopy(T0_w_in)

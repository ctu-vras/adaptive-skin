#!/usr/bin/env python3
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens

This source code is derived from the dmp_gestures project
(https://github.com/awesomebytes/dmp_gestures)
Copyright (c) 2013, Willow Garage, Inc., licensed under the BSD license,
cf. 3rd-party-licenses.txt file in the root directory of this source tree.
"""

"""
Created on 12/08/14
@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com
This file contains kinematics related classes to ease
the use of MoveIt! kinematics services.
"""

"""
Edited by: Lukas Rustler
"""

import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse, \
    GetPositionIK, GetPositionIKRequest, GetPositionIKResponse, \
    GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

DEFAULT_FK_SERVICE = "/compute_fk"
DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_SV_SERVICE = "/check_state_validity"


class ForwardKinematics():
    """Simplified interface to ask for forward kinematics"""

    def __init__(self):
        robot_name = rospy.get_param("/robot_name", "")
        if robot_name != "":
            global DEFAULT_FK_SERVICE
            DEFAULT_FK_SERVICE = "/"+robot_name+DEFAULT_FK_SERVICE
        rospy.loginfo("Loading ForwardKinematics class.")
        self.fk_srv = rospy.ServiceProxy(DEFAULT_FK_SERVICE, GetPositionFK)
        rospy.loginfo("Connecting to FK service")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Ready for making FK calls")

    def closeFK(self):
        self.fk_srv.close()

    def getFK(self, fk_link_names, joint_names, positions, frame_id='base_link'):
        """Get the forward kinematics of a joint configuration
        @fk_link_names list of string or string : list of links that we want to get the forward kinematics from
        @joint_names list of string : with the joint names to set a position to ask for the FK
        @positions list of double : with the position of the joints
        @frame_id string : the reference frame to be used"""
        gpfkr = GetPositionFKRequest()
        if type(fk_link_names) == type("string"):
            gpfkr.fk_link_names = [fk_link_names]
        else:
            gpfkr.fk_link_names = fk_link_names
        gpfkr.robot_state.joint_state.name = joint_names
        gpfkr.robot_state.joint_state.position = positions
        gpfkr.header.frame_id = frame_id
        fk_result = self.fk_srv.call(gpfkr)
        return fk_result

    def getCurrentFK(self, fk_link_names, frame_id='base_link'):
        """Get the forward kinematics of a set of links in the current configuration"""
        # Subscribe to a joint_states
        js = rospy.wait_for_message('/joint_states', JointState)
        # Call FK service
        fk_result = self.getFK(fk_link_names, js.name, js.position, frame_id)
        return fk_result


class InverseKinematics():
    """Simplified interface to ask for inverse kinematics"""

    def __init__(self, ik_srv=None):
        robot_name = rospy.get_param("/robot_name", "")
        if robot_name != "":
            global DEFAULT_IK_SERVICE
            DEFAULT_IK_SERVICE = "/"+robot_name+DEFAULT_IK_SERVICE
        if ik_srv is None:
            self.ik_srv = rospy.ServiceProxy(DEFAULT_IK_SERVICE, GetPositionIK)
        else:
            self.ik_srv = rospy.ServiceProxy(ik_srv, GetPositionIK)
        self.ik_srv.wait_for_service()

    def closeIK(self):
        self.ik_srv.close()

    def getIK(self, group_name, ik_link_name, pose, avoid_collisions=True, robot_state=None,
              constraints=None, timeout=1):
        """Get the inverse kinematics for a group with a link a in pose in 3d world.
        @group_name string group i.e. right_arm that will perform the IK
        @ik_link_name string link that will be in the pose given to evaluate the IK
        @pose PoseStamped that represents the pose (with frame_id!) of the link
        @avoid_collisions Bool if we want solutions with collision avoidance
        @attempts Int number of attempts to get an Ik as it can fail depending on what IK is being used
        @robot_state RobotState the robot state where to start searching IK from (optional, current pose will be used
        if ignored)"""
        gpikr = GetPositionIKRequest()
        gpikr.ik_request.group_name = group_name
        if robot_state != None:  # current robot state will be used internally otherwise
            gpikr.ik_request.robot_state = robot_state
        gpikr.ik_request.avoid_collisions = avoid_collisions
        gpikr.ik_request.ik_link_name = ik_link_name
        if type(pose) == type(PoseStamped()):
            gpikr.ik_request.pose_stamped = pose
        else:
            rospy.logerr("pose is not a PoseStamped, it's: " + str(type(pose)) + ", can't ask for an IK")
            return
        if constraints != None:
            gpikr.ik_request.constraints = constraints
        gpikr.ik_request.timeout = rospy.Duration(timeout)
        ik_result = self.ik_srv.call(gpikr)
        return ik_result.solution.joint_state.position[:7]


class StateValidity():
    def __init__(self):
        rospy.loginfo("Initializing stateValidity class")
        self.sv_srv = rospy.ServiceProxy(DEFAULT_SV_SERVICE, GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        self.sv_srv.wait_for_service()
        if rospy.has_param('/play_motion/approach_planner/planning_groups'):
            list_planning_groups = rospy.get_param('/play_motion/approach_planner/planning_groups')
            # Get groups and joints here
            # Or just always use both_arms_torso...
        else:
            rospy.logwarn("Param '/play_motion/approach_planner/planning_groups' not set. We can't guess controllers")
        rospy.loginfo("Ready for making Validity calls")

    def close_SV(self):
        self.sv_srv.close()

    def getStateValidity(self, robot_state, group_name='both_arms_torso', constraints=None):
        """Given a RobotState and a group name and an optional Constraints
        return the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)

        return result.valid
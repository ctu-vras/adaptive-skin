#!/usr/bin/env python3
"""
Classes for robot movement

Edited from official ROS tutorial by Lukas Rustler
"""
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, msg_to_string
import tf
from sensor_msgs.msg import JointState
import moveit_python
import numpy as np
from bullet_ros.srv import grip, gripRequest


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self, group_):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot_name = rospy.get_param("/robot_name", "")
        self.group_name = group_
        if robot_name != "":
            robot = moveit_commander.RobotCommander(robot_description=robot_name+"/robot_description", ns=robot_name)
            scene = moveit_commander.PlanningSceneInterface(ns=robot_name)
            group = moveit_commander.MoveGroupCommander(self.group_name,
                                                        robot_description=robot_name + "/robot_description",
                                                        ns=robot_name)
            self.planning_scene = moveit_python.planning_scene_interface.PlanningSceneInterface("base_link",
                                                                                                ns=robot_name)
        else:
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            group = moveit_commander.MoveGroupCommander(self.group_name)
            self.planning_scene = moveit_python.planning_scene_interface.PlanningSceneInterface("base_link")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.joint_positions = {}
        tr_exec_topic = '/trajectory_execution_event' if robot_name == "" else ("/"+robot_name+'/trajectory_execution_event')
        self.trajectory_event_publisher = rospy.Publisher(tr_exec_topic, String, queue_size=10)

        self.ls = tf.TransformListener()
        self.js_topic = "/joint_states" if robot_name == "" else ("/"+robot_name+"/joint_states")

        self.gripper_service = rospy.ServiceProxy("/robot/ur_hardware_interface/grip", grip)

    def apply_planning_scene(self, scene_msg):
        return self.scene._psi.apply_planning_scene(msg_to_string(scene_msg))

    def get_current_state(self):
        # stupid workaround to get joint states for the arm and not gripper/finger
        while True:
            msg = rospy.wait_for_message(self.js_topic, JointState)
            if len(msg.name) > 1:
                break
        joint_values = np.array(msg.position)
        # joint_values = joint_values[[2, 1, 0, 3, 4, 5]]
        joint_values = joint_values[:6]
        return joint_values

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal         A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def go_to_joint_position(self, joint_goal, wait=True):

        self.group.go(joint_goal, wait=wait)

        current_joints = self.group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def plan_cartesian_path(self, wpose, collisions=True):
        """ Cartesian Paths
        ^^^^^^^^^^^^^^^
        You can plan a Cartesian path directly by specifying a list of waypoints
        for the end-effector to go through:"""
        if isinstance(wpose, list):
            waypoints = wpose
        else:
            waypoints = [wpose]

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.001,  # eef_step
            0.0,   # jump_threshold
            avoid_collisions=collisions)
        return plan, fraction

    def get_ee_pose(self, scale=1):
        return self.group.get_current_pose().pose

    def display_trajectory(self, plan):
        """ Display trajectory in Rviz, the following code is needed for proper displaying.
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan, wait=True):
        """ **Note:** The robot's current joint state must be within some tolerance of the
         first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail"""
        self.group.execute(plan, wait=wait)

    def stop_robot(self):
        self.group.stop()

    def go_to_pose(self, pose, wait):
        if isinstance(pose, list):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose[0]
            pose_goal.position.y = pose[1]
            pose_goal.position.z = pose[2]
            pose_goal.orientation.x = pose[3]
            pose_goal.orientation.y = pose[4]
            pose_goal.orientation.z = pose[5]
            pose_goal.orientation.w = pose[6]
        elif isinstance(pose, geometry_msgs.msg.Pose):
            pose_goal = pose
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose_goal)
        return self.group.go(wait=wait)

    def open_gripper(self):
        self.gripper_service.call(gripRequest(150, 120))

    def close_gripper(self):
        self.gripper_service.call(gripRequest(0, 120))

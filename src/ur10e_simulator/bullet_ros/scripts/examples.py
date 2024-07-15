#!/usr/bin/env python3

import rospy
from bullet_ros.motion_interface import MoveGroupPythonInterface
from bullet_ros.robot_kinematics_interface import ForwardKinematics, InverseKinematics
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from bullet_ros.srv import grip, gripRequest


if __name__ == "__main__":
    rospy.init_node("examples_node")

    """
    Connect to Motion Interface with group 'manipulator'
        - group 'manipulator' is from base_link to gripper_link frame (in the middle of solid part of the gripper)
        - there is also group 'arm' going to the flange of the robot (tool0 link)
        - other groups can be defined in universal_robot/ur10e_moveit_config/config/ur10e_rg6.srdf
    """

    MoveGroupArm = MoveGroupPythonInterface("manipulator")

    """
    Just info message
    """
    rospy.loginfo("Starting ")

    """
    Get current end-effector and get its pose:
    """
    rospy.loginfo(f"Current end-effector is {MoveGroupArm.eef_link}")
    rospy.loginfo(f"Pose of the end-effector is: {MoveGroupArm.get_ee_pose()}")

    """
    Move the robot 5cm up from its current position
    """
    p = MoveGroupArm.get_ee_pose()
    p.position.z -= 0.1
    MoveGroupArm.go_to_pose(p, wait=True)  # True here is to block the program until completion of the movement.

    """
    Get current joint state of the robot:
    """
    rospy.loginfo(f"Joint state is: {MoveGroupArm.get_current_state()}")

    """
    Move joint_0 25 degrees from current position
    """
    p = MoveGroupArm.get_current_state()
    p[0] += np.deg2rad(25)
    MoveGroupArm.go_to_joint_position(p)

    """
    Open the gripper
    """
    rospy.loginfo("Opening gripper")
    MoveGroupArm.open_gripper()

    """
    Close the gripper
    """
    rospy.loginfo("Closing gripper")
    MoveGroupArm.close_gripper()

    """
    The previous command corresponds to calling /ur_hardware_interface/grip service with width 150mm and force 120N
    for opening and 0mm and 120N for closing
    The service also support other widths (0-150) and forces (0-120).
    To call it
    """
    grip_service = rospy.ServiceProxy("/robot/ur_hardware_interface/grip", grip)
    request = gripRequest()
    request.width = 100
    request.force = 30

    rospy.loginfo(f"Gripper moved successfully: {grip_service.call(request)}")

    """
    Get inverse kinematics without movement
    """
    # Create pose request
    pose_current = MoveGroupArm.get_ee_pose()

    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "base_link"
    ps.pose = pose_current

    ik = InverseKinematics()

    result = ik.getIK(group_name="manipulator", ik_link_name="gripper_link", pose=ps, avoid_collisions=True, timeout=0.1)
    rospy.loginfo(f"The IK solution is: {result}")

    """VELOCITY CONTROL"""
    # Switch to velocity control
    switch_srv = rospy.ServiceProxy("/robot/controller_manager/switch_controller", SwitchController)
    switch_srv.wait_for_service()
    req = SwitchControllerRequest()
    req.start_controllers = ["joint_group_vel_controller"]
    req.stop_controllers = ["scaled_pos_joint_traj_controller"]
    req.strictness = 2
    req.start_asap = False
    switch_srv.call(req)

    # Publish velocity commands
    pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1, latch=True)
    msg = Float64MultiArray()
    msg.data = [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # 0.1 rad/s in joint_0
    pub.publish(msg)

    rospy.sleep(5) # do the movement for some time

    msg.data = [0, 0, 0, 0, 0, 0]  # stop the motion by calling 0 in all
    pub.publish(msg)


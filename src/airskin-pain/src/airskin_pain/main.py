#!/usr/bin/env python3
import os.path
from subprocess import Popen, PIPE
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray
import tf
import PyKDL as kdl
from kdl_parser_py import urdf as kdl_parser
import numpy as np
from sensor_msgs.msg import JointState
import tf.transformations as ts
import datetime
import yaml
import argparse
from airskin_pain.utils import KDLHandle
from std_msgs.msg import Int32


class Experiment:
    INIT_POSE = [0.143, -1.785, 2.148, -0.338, 0.142, -0.032]

    def __init__(self, setup="sim", save_bag=True, config="test.yaml", bag_name=None):
        rospy.init_node("experiment_node")
        # Load correct functions based on the setup
        if setup == "sim":
            from bullet_ros.robot_kinematics_interface import ForwardKinematics
            from bullet_ros.motion_interface import MoveGroupPythonInterface
        elif setup == "real":
            from ur10e_humanoids.robot_kinematics_interface import ForwardKinematics
            from ur10e_humanoids.motion_interface import MoveGroupPythonInterface
        else:
            raise ValueError("setup must be 'sim' or 'real")
        self.setup = setup

        # Prepare bag folder and run the recording
        if save_bag:
            if bag_name is None:
                start_time_ = datetime.datetime.now()
                bag_name = str(start_time_).replace(".", "-").replace(" ", "-").replace(":", "-")
            bag_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "bags", bag_name + ".bag")
            if not os.path.exists(os.path.dirname(bag_path)):
                os.makedirs(os.path.dirname(bag_path))
            command = f"rosbag record -O {bag_path} -a __name:=my_bag"
            Popen(command, stdout=PIPE, shell=True)

        # FK service
        self.fk = ForwardKinematics()

        # controller mode switch service
        self.switch_srv = rospy.ServiceProxy("/robot/controller_manager/switch_controller", SwitchController)
        self.switch_srv.wait_for_service()

        # tf listener -> it is better to have one during the whole time
        self.listener = tf.TransformListener()

        # publisher for velocity control
        self.velocity_publisher = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1, latch=True)
        self.velocity_msg = Float64MultiArray()

        try:
            # python move_group commander interface
            self.mg = MoveGroupPythonInterface("manipulator")
        except:
            self.mg = MoveGroupPythonInterface("manipulator")

        # joint states subscriber
        self.js = None
        self.js_vel = None
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self.get_js)
        self.joint_names = rospy.wait_for_message("/joint_states", JointState).name[:6]

        # kdl init
        _, self.kdl_tree = kdl_parser.treeFromParam("robot_description")
        self.kdl_handles = {}
        # Add airskin links
        for pad in range(11):
            self.kdl_handles[f"airskin_{pad}"] = KDLHandle(f"airskin_{pad}", self.kdl_tree)
        # add the end-effector
        self.kdl_handles["gripper"] = KDLHandle("gripper_link", self.kdl_tree)

        # Generic properties
        self.phase = 0
        self.js_when_impact = None
        self.js_vel_when_impact = None
        self.last_activated_airskin = None
        self.phase_when_impact = None
        self.max_velocity = 0.5 if setup == "real" else 0.5
        self.min_velocity = 0.05
        self.link_velocity = self.max_velocity
        self.end_points = []

        config = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "configs", config)
        with open(config, "r") as f:
            config = yaml.safe_load(f)
            self.behaviour = config["behaviour"]
            self.thresholds = config["thresholds"]
            if config["mode"] in ["mass", "norm"]:
                self.thresholds = []

        # airskin processing
        self.pain = None
        self.airskin_sub = rospy.Subscriber("/airskin_pain/touch", Int32, self.airskin_cb, queue_size=1)
        self.airskin_act_time = None

        # status publisher
        self.status_pub = rospy.Publisher("/airskin_pain/phase", Int32, queue_size=1, latch=True)

        self.axes = [-1, 1, 2, 2, 0, 2, 2]
        self.generate_end_points()
        self.avoid_direction = None
        # Actually important on slower computers
        self.rate = rospy.Rate(100)

    def __del__(self):
        """
        Destructor to ensure bag recording will end all the time

        :return:
        :rtype:
        """
        proc = Popen("rosnode kill /my_bag", shell=True, stdout=PIPE, stderr=PIPE)
        proc.wait()

    def airskin_cb(self, msg):
        """
        Callback for airskin event

        :param msg: message with the airskin event
        :type msg: std_msgs/Int32
        :return:
        :rtype:
        """
        # touch
        if msg.data != -1:
            self.pain = msg.data
            self.last_activated_airskin = self.pain
            self.phase_when_impact = self.phase
            self.phase = -2
            self.js_when_impact = np.array(self.js)
            self.js_vel_when_impact = np.array(self.js_vel)
        # end of touch
        else:
            self.pain = None
            self.phase = -1

    def generate_end_points(self):
        """
        Help function to generate trajectories end-points to know when movement ended

        :return:
        :rtype:
        """
        self.end_points.append(None)
        start_pose = self.fk.getFK("gripper_link", self.joint_names, self.INIT_POSE)
        start_point = np.array([getattr(start_pose.pose_stamped[0].pose.position, _) for _ in ["x", "y", "z"]])

        offsets = [[0, 0.6, 0], [0, 0, -0.35], [0, 0, 0.35], [-0.75, 0, 0], [0, 0, -0.35], [0, 0, 0.35]]
        end_point = start_point
        for offset in offsets:
            end_point = end_point + offset
            self.end_points.append(end_point)

    def get_js(self, msg):
        """
        Callback for joint state msg
        :param msg: message with joint states
        :type msg: sensor_msgs/JointState
        :return:
        :rtype:
        """
        self.js = msg.position[:6]
        self.js_vel = msg.velocity[:6]

    def compute_velocity(self, kdl_handle, axis, local=True):
        """

        :param kdl_handle: pointer to instance of kdl handle class that encapsulates kdl function for given chain
        :type kdl_handle: KDLHandle or None
        :param axis: if local = True: axis in the local frame of the end-effector which we should follow;
                                      its will be base for the translation velocity
                     if local = False: the direction of movement in base_link frame
        :type axis: list 1x3
        :param local: whether the axis is expressed on local (end-effector) or base frame
        :type local: bool
        :return:
        :rtype:
        """

        if local:
            # Get current rotation of the pad and compute its negative normal
            _, rot = self.get_transformation(kdl_handle.end_effector, "base_link")

            direction = np.matmul(ts.quaternion_matrix(rot), np.hstack((axis, 1)))[
                        :3]  # go against normal of the given link
            direction /= np.linalg.norm(direction)  # normalize
            # Set goal speed to 10 cm/s against the normal
        else:
            direction = np.array(axis)

        return self.link_velocity*direction

    def move_velocity(self, kdl_handle, velocity):
        """
        Function to compute velocity of joints using Jacobian matrix for given end-effector with given axis of motion
        :param kdl_handle: pointer to instance of kdl handle class that encapsulates kdl function for given chain
        :type kdl_handle: KDLHandle
        :param velocity: translational velocity of the given end-effector
        :return: 1x3 list
        :rtype:
        """
        # create goal for KDL with zero angular velocity
        goal = kdl.Twist(kdl.Vector(*velocity), kdl.Vector(*[0, 0, 0]))

        joints = kdl_handle.np_to_kdl(self.js[:kdl_handle.num_joints])

        # Solve -> the output will be saved directly to velocities
        kdl_handle.vel_solver.CartToJnt(joints, goal, kdl_handle.velocities)
        # Publish velocity commands
        self.velocity_msg.data = kdl_handle.kdl_to_np(kdl_handle.velocities).tolist() + [0] * (6 - kdl_handle.num_joints)  # 0.1 rad/s in joint_0
        self.velocity_publisher.publish(self.velocity_msg)

    def is_close(self):
        """
        Checks whether the end-effector is close to the desired position
        :return:
        :rtype:
        """
        pose = self.mg.get_ee_pose()
        position = [getattr(pose.position, _) for _ in ["x", "y", "z"]]
        if self.phase == -1:
            return False
        dist = np.abs(position[self.axes[self.phase]] - self.end_points[self.phase][self.axes[self.phase]])
        self.link_velocity = np.max([self.max_velocity * np.min([dist*12.5, 1]), self.min_velocity])
        return dist < 0.0075

    def run(self):
        """
        Main function of the class
        :return:
        :rtype:
        """

        while not rospy.is_shutdown():
            self.status_pub.publish(Int32(data=self.phase))

            # airskin signal
            if self.phase == -2:
                if self.behaviour == "avoid":
                    if True:  # remnant from older version, but I am too lazy to delete it
                        if self.avoid_direction is None:  # compute where to go with the end-effector
                                h = self.kdl_handles[f"airskin_{self.last_activated_airskin}"]
                                pos_arr = h.np_to_kdl(self.js_when_impact[:h.num_joints])
                                vel_arr = h.np_to_kdl(self.js_vel_when_impact[:h.num_joints])
                                h.fk_vel_solver.JntToCart(kdl.JntArrayVel(pos_arr, vel_arr), h.fk_vel_frame)
                                u = h.kdl_to_np(h.fk_vel_frame.p.v, 3)
                                u_norm = np.linalg.norm(u)
                                u /= u_norm
                                self.avoid_direction = u
                        velocity = self.compute_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"], -self.avoid_direction, False)
                    else:
                        velocity = self.compute_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"], [0, 0, -1], True)
                else:
                    # in case of STOP just stop
                    velocity = [0, 0, 0]
                self.move_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"], velocity)

            # phase to return back to path
            elif self.phase == -1:
                if self.behaviour == "avoid":
                    if self.avoid_direction is not None:  # go in other direction then we were going
                        self.link_velocity = self.max_velocity / 2
                        velocity = self.compute_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"], self.avoid_direction, False)
                    else:  # go in the direction of the normal of airskin (never used in this version)
                        velocity = self.compute_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"],[0, 0, 1], True)
                    self.move_velocity(self.kdl_handles[f"airskin_{self.last_activated_airskin}"], velocity)
                    if np.all(np.abs(self.js - self.js_when_impact) < 0.05):  # check that we are back
                        self.phase = self.phase_when_impact
                        self.avoid_direction = None
                        self.link_velocity = self.max_velocity
                else:
                    self.phase = self.phase_when_impact
            # go home
            elif self.phase == 0:
                if self.setup == "sim":
                    self.mg.close_gripper()
                self.switch_controllers(velocity=False)
                self.mg.go_to_joint_position(self.INIT_POSE)
                self.switch_controllers(velocity=True)
                self.phase = 1
            # forward movement
            elif self.phase == 1:
                velocity = self.compute_velocity(None, [0, 1, 0], False)
                self.move_velocity(self.kdl_handles["gripper"], velocity)
                if self.is_close():
                    self.phase = 2
            # down
            elif self.phase == 2 or self.phase == 5:
                velocity = self.compute_velocity(None, [0, 0, -1], False)
                self.move_velocity(self.kdl_handles["gripper"], velocity)
                if self.is_close():
                    if self.phase == 2:
                        self.phase = 3
                    else:
                        self.phase = 6
            # up
            elif self.phase == 3 or self.phase == 6:
                velocity = self.compute_velocity(None, [0, 0, 1], False)
                self.move_velocity(self.kdl_handles["gripper"], velocity)
                if self.is_close():
                    if self.phase == 3:
                        self.phase = 4
                    else:  # actually now it will never go here
                        # end here
                        self.move_velocity(self.kdl_handles["gripper"], [0, 0, 0])
                        self.status_pub.publish(Int32(data=99))
                        break  # self.phase = 0
            # left
            elif self.phase == 4:
                velocity = self.compute_velocity(None, [-1, 0, 0], False)
                self.move_velocity(self.kdl_handles["gripper"], velocity)
                if self.is_close():
                    # self.phase = 5
                    # end here
                    self.move_velocity(self.kdl_handles["gripper"], [0, 0, 0])
                    self.status_pub.publish(Int32(data=99))
                    break  # self.phase = 0
            self.rate.sleep()

    def get_transformation(self, what, where):
        """
        Help util to get transformation
        @param what: For what frame to obtain the transformation
        @type what: string
        @param where: In which frame to express
        @type where: string
        @return:
        @rtype:
        """
        translation, rotation = self.listener.lookupTransform(where, what, rospy.Time(0))
        return np.array(translation), np.array(rotation)

    def switch_controllers(self, velocity=True):
        """
        Function to call service to switch robot's controllers

        Right now, the function does not check whether the desired controller is already running -> it can produce
        error message in ROS output and return False
        :param velocity: whether to switch to velocity control
        :type velocity: bool
        :return: success status
        :rtype: bool
        """
        req = SwitchControllerRequest()
        if velocity:
            req.start_controllers = ["joint_group_vel_controller"]
            req.stop_controllers = ["scaled_pos_joint_traj_controller"]
        else:
            req.start_controllers = ["scaled_pos_joint_traj_controller"]
            req.stop_controllers = ["joint_group_vel_controller"]
        req.strictness = 2
        req.start_asap = False
        return self.switch_srv.call(req)


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )
    arg_parser.add_argument(
        "--setup",
        "-s",
        dest="setup",
        required=False,
        default="sim",
        help="Which setup to use: sim or real"
    )

    arg_parser.add_argument(
        "--save_bag",
        "-b",
        dest="save_bag",
        action="store_true",
        required=False,
        default=False,
        help="Whether to save the bag file"
    )

    arg_parser.add_argument(
        "--config",
        "-c",
        dest="config",
        required=False,
        default="test.yaml",
        help="Path to the configuration file"
    )

    arg_parser.add_argument(
        "--bag_name",
        "-bn",
        dest="bag_name",
        required=False,
        default=None,
        help="Name of the bag to save"
    )

    args = arg_parser.parse_args()
    return args.setup, args.save_bag, args.config, args.bag_name


if __name__ == "__main__":
    setup, save_bag, config, bag_name = prepare_parser()
    e = Experiment(setup, save_bag, config, bag_name)
    e.run()

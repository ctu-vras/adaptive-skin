#!/usr/bin/env python3
"""
Node to calculate effective mass of given links and publish thresholds based on force computed.

:Author: Lukas Rustler
"""
from airskin_pain.utils import KDLHandle
import rospy
from kdl_parser_py import urdf as kdl_parser
from sensor_msgs.msg import JointState
import numpy as np
import PyKDL as kdl
from std_msgs.msg import Int32MultiArray
import argparse
import sys


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )

    arg_parser.add_argument(
        "--mode",
        "-m",
        dest="mode",
        required=False,
        default="mass",
        help="Mode of computation: mass or norm"
    )

    args_to_parse = []
    for arg in sys.argv[1:]:
        if ("__" and ":=" and "_") not in arg:
            args_to_parse.append(arg)

    args = arg_parser.parse_args(args_to_parse)
    return args.mode


class Mass:
    def __init__(self, mode):
        self.mode = mode
        # kdl init
        _, self.kdl_tree = kdl_parser.treeFromParam("robot_description")
        self.kdl_handles = {}
        # Add airskin links
        for pad in range(11):
            self.kdl_handles[f"airskin_{pad}"] = KDLHandle(f"airskin_{pad}", self.kdl_tree)
        if self.mode == "norm":
            self.gripper = KDLHandle("gripper_link", self.kdl_tree)

        # Force levels and thresholds
        self.force_levels = np.array([280, 140, 0])
        self.thresholds = Int32MultiArray(data=[0]*11)

        # Thresholds publisher
        self.pub = rospy.Publisher("/airskin_pain/thresholds", Int32MultiArray, queue_size=1, latch=True)

        # Joint states values init
        self.js = None
        self.js_vel = None
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self, queue_size=1)

    def __call__(self, msg):
        """
        Callback for joint states

        :param msg:
        :type msg:
        :return:
        :rtype:
        """
        self.js = np.array(msg.position)
        self.js_vel = np.array(msg.velocity)

    def run(self):
        """
        Main loop that computes effective mass, Force and publishes thresholds

        :return:
        :rtype:
        """
        while not rospy.is_shutdown():
            if self.js is not None:
                for h_id, h in enumerate(self.kdl_handles.values()):
                    js = self.js
                    js_vel = self.js_vel

                    # Get current cartesian velocity
                    pos_arr = h.np_to_kdl(js[:h.num_joints])
                    vel_arr = h.np_to_kdl(js_vel[:h.num_joints])
                    h.fk_vel_solver.JntToCart(kdl.JntArrayVel(pos_arr, vel_arr), h.fk_vel_frame)

                    u = h.kdl_to_np(h.fk_vel_frame.p.v, 3)
                    u_norm = np.linalg.norm(u)
                    if u_norm != 0:
                        u /= u_norm

                        # Compute eff mass
                        if self.mode == "mass":
                            h.jac_solver.JntToJac(h.np_to_kdl(js[:h.num_joints]), h.jac)
                            h.dyn_solver.JntToMass(h.np_to_kdl(js[:h.num_joints]), h.dyn)
                            M = h.kdl_to_np_mat(h.dyn)
                            J = h.kdl_to_np_mat(h.jac)

                            Ai = J @ (np.linalg.inv(M) @ J.T)
                            Avi = Ai[0:3, 0:3]  # == J[:3, :] @ (np.linalg.inv(M) @ J[:3, :].T)

                            m = 1 / (u.T @ Avi @ u)
                            if m > h.mass:
                                m = 0
                        # Mass if moving mass/2
                        else:
                            m = h.mass/2
                        # Some edge cases
                        if m == 0:
                            F = 0
                        else:
                            F = (u_norm * np.sqrt(75000)) /np.sqrt(1/m + 1/5.6)
                        if F < 0:
                            F = 0
                    else:
                        F = 0
                    # get the right threshold
                    self.thresholds.data[h_id] = np.nonzero(F >= self.force_levels)[0][0]
                # publish
                self.pub.publish(self.thresholds)


if __name__ == "__main__":
    rospy.init_node("eff_mass_calculator")
    mode = prepare_parser()
    m = Mass(mode)
    m.run()

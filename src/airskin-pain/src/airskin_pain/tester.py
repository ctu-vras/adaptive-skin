#!/usr/bin/env python3
"""
Script to touch the airskin given config file.

:Author: Lukas Rustler
"""

import rospy
from std_msgs.msg import Int32
import yaml
import os
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
import argparse


class Tester:
    def __init__(self, setup="sim", config="test.yaml"):
        rospy.init_node("airskin_pain_tester")
        if setup == "sim":
            from bullet_ros.srv import changeObjPoseRequest, changeObjPose
            from bullet_ros.srv import activateAirskin, activateAirskinRequest
        elif setup == "real":
            pass
        else:
            raise ValueError("setup must be 'sim' or 'real")
        # Init variables
        self.setup = setup
        self.phase_sub = rospy.Subscriber("/airskin_pain/phase", Int32, self, queue_size=1)
        self.hit_pub = rospy.Publisher("/airskin_pain/action", Int32, queue_size=1, latch=True)
        self.phase = None
        self.last_phase = None
        self.robot_phase = None
        self.start_time = rospy.get_time()
        self.phases_start = [None]*7
        config = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "configs", config)
        with open(config, "r") as f:
            self.config = yaml.safe_load(f)
        self.config_id = 0
        # Load config and put some variables in the class for quicker access
        if len(self.config["phases"]) > 0:
            self.phase_to_touch = self.config["phases"][self.config_id]
            self.touch_time = self.config["times"][self.config_id]
            self.follow = self.config["follow"][self.config_id]
            self.behaviour = self.config["behaviour"]
        else:
            self.phase_to_touch = -2
        self.hit = False

        if setup == "sim":
            # Ball pose changing service to "hit" airskin
            self.change_ball_srv = rospy.ServiceProxy("/bullet_ros/change_obj_pose", changeObjPose)
            self.change_ball_req = changeObjPoseRequest()

            # srv to hit airskin
            self.activate_airskin_srv = rospy.ServiceProxy("/bullet_ros/activate_airskin", activateAirskin)
            self.activate_airskin_req = activateAirskinRequest()

        # tf listener -> it is better to have one during the whole time
        self.listener = tf.TransformListener()

        self.next_touch_locked = False

    def __call__(self, msg):
        """
        Callback for robot phase

        :param msg:
        :type msg:
        :return:
        :rtype:
        """
        self.robot_phase = msg.data

    def next_touch(self):
        """
        Function to handle next touch -> set new variables

        :return:
        :rtype:
        """
        self.next_touch_locked = True
        self.config_id += 1
        if self.config_id >= len(self.config["phases"]):
            self.phase_to_touch = -2
            return 0
        self.phase_to_touch = self.config["phases"][self.config_id]
        self.touch_time = self.config["times"][self.config_id]
        self.follow = self.config["follow"][self.config_id]

    def run(self):
        """
        Main loop

        :return:
        :rtype:
        """
        while not rospy.is_shutdown():
            # Break if the phase is not changing for 60 seconds -> failed run from some reason
            if self.phases_start[1] is not None and rospy.get_time() - self.phases_start[1] > 60:
                break
            # Task ended -> end this as well
            if self.robot_phase == 99:
                break

            # Robot changed the phase
            if self.robot_phase != self.last_phase:

                # robot changed phase (when airskin activated)
                if self.hit and self.robot_phase != -2:
                    self.hit_pub.publish(-1)
                    self.hit_airskin(-1)
                    if not self.next_touch_locked:
                        self.next_touch()
                    self.hit = not self.hit

                # robot changed phase (when airskin not activated)
                if self.robot_phase not in [-1, -2, 99]:
                    self.phase = self.robot_phase
                    if self.phases_start[self.phase] is None:
                        self.phases_start[self.phase] = rospy.get_time()

                self.last_phase = self.robot_phase
            if self.phase == self.phase_to_touch:
                if (rospy.get_time() - self.phases_start[self.phase]) > self.touch_time:
                    # activate when the time is right based on config
                    if not self.hit:
                        self.next_touch_locked = False
                        self.hit_pub.publish(self.config["pads"][self.config_id])
                        self.hit_airskin(self.config["pads"][self.config_id], self.config["efforts"][self.config_id],
                                         self.follow, self.config["increase_effort"][self.config_id])
                        self.touch_time += self.config["durations"][self.config_id]
                    # deactivate -> useful only for STOP
                    else:
                        self.hit_pub.publish(-1)
                        self.hit_airskin(-1)
                        if not self.next_touch_locked:
                            self.next_touch()
                    self.hit = not self.hit

    def hit_airskin(self, pad, effort=1.0, follow=False, increase_effort=False):
        """
        Help function to call hit_airskin service

        :param pad: pad number
        :type pad: int
        :param effort: effort
        :type effort: float
        :param follow: whether to follow the pad (stay always close)
        :type follow: bool
        :param increase_effort: whether to gradually increase the effort
        :type increase_effort: bool
        :return:
        :rtype:
        """
        if self.setup == "sim":
            self.activate_airskin_req.pad_id = pad
            self.activate_airskin_req.effort = effort
            self.activate_airskin_req.follow = follow
            self.activate_airskin_req.increase_effort = increase_effort
            self.activate_airskin_srv.call(self.activate_airskin_req)
        else:
            if pad != -1:
                print("Touch")
            else:
                print("Stop touching")

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
        "--config",
        "-c",
        dest="config",
        required=False,
        default="test.yaml",
        help="Path to the configuration file"
    )

    args = arg_parser.parse_args()
    return args.setup, args.config


if __name__ == "__main__":
    setup, config = prepare_parser()
    t = Tester(setup, config)
    t.run()

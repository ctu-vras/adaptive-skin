#!/usr/bin/env python3
"""
Node with airskin feedback. Runs the correct utils and sends event feedback.

:Author: Lukas Rustler
"""
import rospy
from airskin_pain.utils import AirskinFeedbackStandAlone
import argparse
import os
import yaml
from std_msgs.msg import Int32MultiArray


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
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
        "--setup",
        "-s",
        dest="setup",
        required=False,
        default="sim",
        help="Setup"
    )

    args = arg_parser.parse_args()
    return args.config, args.setup


if __name__ == "__main__":
    config, setup = prepare_parser()
    config = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "configs", config)
    with open(config, "r") as f:
        config = yaml.safe_load(f)
    behaviour = config["behaviour"]
    thresholds = config["thresholds"]
    if config["mode"] in ["mass", "norm"]:
        thresholds = []

    if setup == "real":
        from airskin.msg import AirskinStatus
    else:
        from bullet_ros.msg import AirskinStatus

    rospy.init_node("airskin_node")
    airskin_detector = AirskinFeedbackStandAlone(setup, thresholds, 30, 0)
    airskin_detector.sub = rospy.Subscriber("/airskin_status", AirskinStatus, airskin_detector, queue_size=1)
    if len(thresholds) == 0:
        airskin_detector.thr_sub = rospy.Subscriber("/airskin_pain/thresholds", Int32MultiArray, airskin_detector.thr_cb, queue_size=1)

    rospy.spin()

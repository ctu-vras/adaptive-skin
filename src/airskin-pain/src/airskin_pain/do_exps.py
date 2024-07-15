#!/usr/bin/env python3
"""
Main experiment runner. Runs all necessary things based on config file.

:Author: Lukas Rustler
"""
from subprocess import call, PIPE, Popen
import time
import argparse
import os
import yaml
import datetime
from pathlib import Path


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
        "--offscreen",
        "-o",
        dest="offscreen",
        action="store_true",
        required=False,
        default=False,
        help="Whether to run without RVIZ"
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
        "--iterations",
        "-i",
        dest="iterations",
        required=False,
        default=10,
        help="number of iteration per config file"
    )

    arg_parser.add_argument(
        "--exp_name",
        "-e",
        dest="exp_name",
        required=False,
        default=None,
        help="Name of the experiment"
    )

    args = arg_parser.parse_args()
    return args.setup, "--save_bag" if args.save_bag else "", args.config, args.exp_name, args.offscreen, int(args.iterations)


def create_ts():
    start_time_ = datetime.datetime.now()
    return str(start_time_).replace(".", "-").replace(" ", "-").replace(":", "-"), start_time_


if __name__ == "__main__":
    setup, save_bag, config_name, exp_name, offscreen, iterations = prepare_parser()

    # Create output folder and file
    start_ts, _ = create_ts()
    if exp_name is None:
        exp_name = start_ts
    log_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "exps", exp_name, f"{start_ts}.csv")
    if not os.path.exists(os.path.dirname(log_path)):
        os.makedirs(os.path.dirname(log_path))

    with open(log_path, "w") as f:
        f.write("timestamp;config_path;setup;iteration;bag_name\n")

    # Load config/configs
    config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..", "data", "configs", config_name)
    if os.path.isdir(config_path):
        configs = [os.path.normpath(_) for _ in Path(config_path).rglob("*.yaml")]
    else:
        configs = [config_path]

    for config_path in configs:
        config_name = config_path.split("data/configs/")[-1]
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        for i in range(iterations):
            iteration_start_ts, start_time = create_ts()
            # Run the correct launch
            if setup == "sim":
                cmd = (f"roslaunch airskin_pain main.launch sim:={'true' if setup == 'sim' else 'false'} "
                       f"rviz:={'true' if not offscreen else 'false'}")
                launch = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
                time.sleep(5)
            else:
                print(f"Starting iteration {i} of {config_name} in {setup} setup. "
                      f"Pad to be touched {config['pads']} in phase {config['phases']}.")
                input("Ready?")

            # Run effective mass calculation and threshold publishing script
            if config["mode"] in ["norm", "mass"]:
                cmd = f"rosrun airskin_pain eff_mass.py --mode {config['mode']}"
                mass = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)

            # All the nodes
            cmd = f"rosrun airskin_pain airskin_node.py --config {config_name} --setup {setup}"
            airskin = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)

            cmd = f"rosrun airskin_pain main.py --setup {setup} --config {config_name} {save_bag} --bag_name {iteration_start_ts}"
            mainp = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)

            # Show output just from this one
            cmd = f"rosrun airskin_pain tester.py --setup {setup} --config {config_name}"
            call(cmd, shell=True)

            with open(log_path, "a") as f:
                f.write(f"{iteration_start_ts};{config_name};{setup};{i};{iteration_start_ts}.bag\n")

            # Kill everything
            if setup == "sim":
                call("pkill -f ros", shell=True)
                call("pkill -f airskin_node.py", shell=True)
                call("pkill -f eff_mass.py", shell=True)
                call("pkill -f main.py", shell=True)
                call("pkill -f tester.py", shell=True)
                time.sleep(10)
            print(f"Finished iteration {i} of {config_name} in {setup} setup in {datetime.datetime.now()-start_time}s.")
"""
:Author: Lukas Rustler
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "pyUR"))
from pyur import pyUR
from utils import Pose


def move(client):
    # Get current pose
    cur_pose = client.end_effector.get_position()

    # Copy, the pose and go 15cm lower
    new_pose = Pose(cur_pose.pos, cur_pose.ori)
    new_pose.pos[2] -= 0.4

    # move
    client.move_cartesian(new_pose)

    # get current pose
    pose = client.end_effector.get_position()
    # go back
    pose.pos[2] += 0.4
    # move; do not wait for completion; and move a bit faster
    client.move_cartesian(pose, wait=False, velocity=2)
    # wait manually
    while not client.motion_done():
        client.update_simulation()

    """
    Wait could be also achieved with:
    client.wait_motion_done()
    """

    client.logger.info("Test finished!")


if __name__ == "__main__":
    # load the robot with correct world/config
    client = pyUR(config="default.yaml")

    # move the robot
    move(client)

    # just wait until the gui is closed
    while client.is_alive():
        client.update_simulation()

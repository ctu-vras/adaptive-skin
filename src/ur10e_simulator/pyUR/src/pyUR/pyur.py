import glob
import time
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient
import os

try: # ROS version
    from pyUR.visualizer import Visualizer
    from pyUR.gripper import RG6
    from pyUR.utils import Config, URDF, Pose, CustomFormatter
except ImportError: # Non-ROS version
    from visualizer import Visualizer
    from gripper import RG6
    from utils import Config, URDF, Pose, CustomFormatter

import open3d as o3d
import logging
import datetime
import inspect
from subprocess import call
import atexit


class pyUR(BulletClient):
    """
    Client class which inherits from BulletClient and contains the whole simulation functionality

    """
    # As dict, because IntEnum is about 1.5-2x slower
    jointInfo = {name: i for i, name in enumerate(["INDEX", "NAME", "TYPE", "QINDEX", "UINDEX", "FLAGS", "DAMPING",
                                                   "FRICTION", "LOWERLIMIT", "UPPERLIMIT", "MAXFORCE", "MAXVELOCITY",
                                                   "LINKNAME", "AXIS", "PARENTPOS", "PARENTORN", "PARENTINDEX"])}
    jointStates = {name: i for i, name in enumerate(["POSITION", "VELOCITY", "FORCES", "TORQUE"])}

    linkInfo = {name: i for i, name in enumerate(["WORLDPOS", "WORLDORI", "INERTIAPOS", "INERTIAORI", "URDFPOS",
                                                  "URDFORI", "LINVEL", "ANGVEL"])}

    contactPoints = {name: i for i, name in enumerate(["FLAG", "IDA", "IDB", "INDEXA", "INDEXB", "POSITIONA",
                                                       "POSITIONB", "NORMAL", "DISTANCE", "FORCE",
                                                       "FRICTION1", "FRICTIONDIR1", "FRICTION2", "FRICTIONDIR2"])}
    dynamicsInfo = {name: i for i, name in enumerate(["MASS", "FRICTION", "INTERTIADIAGONAL", "INERTIAPOS", "INERTIAOR",
                                                      "RESTITUTION", "ROLLINGFRICTION", "SPINNINGFRICTION", "DAMPING",
                                                      "STIFFNESS", "BODYTYPE", "MARGIN"])}
    visualShapeData = {name: i for i, name in enumerate(["ID", "LINK", "GEOMTYPE", "DIMS", "FILE", "POS", "ORI",
                                                         "COLOR", "TEXTURE"])}

    def __init__(self, config="default.yaml"):
        """
        :param config: path to the config file
        :type config: str, optional, default="default.yaml"
        """
        super().__init__(p.DIRECT)

        try:
            self.parent_name = os.path.basename(inspect.stack()[1][0].f_locals["__file__"])
        except KeyError:
            self.parent_name = "main.py" # ROS variant
        self.file_dir = os.path.dirname(os.path.abspath(__file__))
        self.config = Config(os.path.join(self.file_dir, "../../configs", config))
        self.config.simulation_step = 1/self.config.simulation_step
        self.setTimeStep(self.config.simulation_step)
        self.gui = self.config.gui

        if self.gui:
            atexit.register(self.kill_open3d)

        self.logger = logging.getLogger("pycub_logger")
        self.logger.setLevel(logging.DEBUG if self.config.debug else logging.INFO)
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(CustomFormatter())
        self.logger.addHandler(stream_handler)

        self.gravity = False
        self.last_skin_step = time.time()

        self.urdf_path = os.path.join(self.file_dir, self.config.robot_urdf_path)

        self.urdfs = {"robot": URDF(self.urdf_path)}

        self.other_objects = []

        if hasattr(self.config, "urdfs"):
            for obj_id, urdf, fixed, color in zip(np.arange(len(self.config.urdfs.paths)), self.config.urdfs.paths, self.config.urdfs.fixed, self.config.urdfs.color):
                suffix = ""
                if os.path.basename(urdf).split(".")[-1] == "obj":
                    obj_name = os.path.basename(urdf).split(".")[0]
                    while obj_name in self.urdfs:
                        suffix += "_"
                        obj_name = obj_name+suffix
                    self.create_urdf(urdf, fixed, color, suffix)
                    urdf = os.path.normpath(os.path.join(self.file_dir, "../../..", "other_meshes", urdf.replace(".obj", suffix+".urdf")))
                elif os.path.basename(urdf).split(".")[-1] == "urdf":
                    urdf = os.path.normpath(os.path.join(self.file_dir, "../../..", "other_meshes", urdf))
                else:
                    raise ValueError("Objects must be .obj or .urdf!")
                self.urdfs[os.path.basename(urdf).split(".")[0]] = URDF(urdf)
                self.config.urdfs.paths[obj_id] = urdf

        if self.config.vhacd.use_vhacd:
            self.run_vhacd()
        self.free_objects = []
        if hasattr(self.config, "urdfs"):
            for urdf, pos, color, fixed in zip(self.config.urdfs.paths, self.config.urdfs.positions, self.config.urdfs.color, self.config.urdfs.fixed):
                obj_name = os.path.basename(urdf).split(".")[0]
                urdf = self.urdfs[obj_name].path
                self.other_objects.append((self.loadURDF(urdf, pos), obj_name, fixed, color, urdf.split("other_meshes/")[1].replace(".urdf", ".obj")))
                if not fixed:
                    self.free_objects.append(self.other_objects[-1][0])

        self.urdf_path = self.urdfs["robot"].path
        self.robot, self.joints, self.links = self.init_robot()

        # prepare IK config so we can utilize null space
        # prepare IK config so we can utilize null space
        self.IK_config = {"movable_joints": [_.joints_id for _ in self.joints if "_hand_" not in _.name],
                          "lower_limits": [_.lower_limit for _ in self.joints],
                          "upper_limits": [_.upper_limit for _ in self.joints],
                          "joint_ranges": [np.abs(_.upper_limit - _.lower_limit) for _ in self.joints],
                          "rest_poses": [0 if not hasattr(self.config.initial_joint_angles, _.name)
                                         else np.deg2rad(getattr(self.config.initial_joint_angles, _.name))
                                         for _ in self.joints]}

        self.end_effector = EndEffector(self.config.end_effector, self)

        self.last_step = time.time()
        self.last_log = time.time()
        self.joint_tolerance = float(self.config.tolerance.joint)

        self.neighbour_links = {}
        if self.config.skin.use:
            self.config.skin.period = 1/self.config.skin.period
            self.activated_skin = {}
            visualData = self.getVisualShapeData(self.robot)
            for m in visualData:
                # Get information about individual parts of the object
                f_path = m[self.visualShapeData["FILE"]].decode("utf-8")
                if "airskin" in f_path and "invisible" not in f_path:
                    if self.config.show_collision:
                        f_path = f_path.replace("visual", "collision").replace(".obj", "_vhacd.obj")
                        self.activated_skin[f_path] = 0

            self.link_to_skin_string = {"upper_arm_link": ["airskin_0", "airskin_1", "airskin_1__1", "airskin_1__2",
                                                           "airskin_2", "airskin_2__1", "airskin_2__2", "airskin_3"],
                                 "forearm_link": ["airskin_4", "airskin_5", "airskin_5__1", "airskin_5__2",
                                                  "airskin_6", "airskin_6__1", "airskin_6__2", "airskin_7"],
                                 "wrist_1_link": ["airskin_8"],
                                 "wrist_2_link": ["airskin_9"],
                                 "wrist_3_link": ["airskin_10"], "onrobot_rg6_base_link": ["airskin_10"],
                                 "left_outer_knuckle": ["airskin_10"], "right_outer_knuckle": ["airskin_10"],
                                 "left_inner_knuckle": ["airskin_10"], "right_inner_knuckle": ["airskin_10"],
                                 "left_inner_finger": ["airskin_10"], "right_inner_finger": ["airskin_10"]}
            self.link_to_skin = {}
            for link, airskin_links in self.link_to_skin_string.items():
                link_id = self.get_link_id(link)
                airskin_links_ids = [self.get_link_id(_) for _ in airskin_links]
                self.link_to_skin[link_id] = (link, airskin_links_ids)
            self.sorted_skin_links = sorted(list(self.activated_skin.keys()), key=lambda x: int(x.replace("_vhacd", "").replace(".obj", "").split("_")[-1]))
        if self.config.log.log:
            self.file_logger = logging.getLogger("pyur_file_logger")
            self.file_logger.setLevel(logging.INFO)
            file_handler = logging.FileHandler(os.path.join(self.file_dir, "../../logs",
                                                            str(datetime.datetime.now()).replace(".", "-").replace(" ", "-")
                                                            .replace(":", "-")+".csv"),
                                               mode="a")
            file_handler.setFormatter(logging.Formatter('%(message)s'))
            self.file_logger.addHandler(file_handler)
            initial_string = "timestamp;steps_done;"+";".join([_.name for _ in self.joints])

            if self.config.skin.use:
                initial_string += ";"+";".join(sorted([os.path.basename(_).split(".obj")[0].split("_vhacd")[0] for _ in self.activated_skin.keys()], key=lambda x: int(x.split("_")[-1])))
            self.file_logger.info(initial_string)

        if self.gui:
            self.visualizer = Visualizer(self)
            self.last_render = time.time()

        self.collision_during_motion = False
        self.steps_done = 0
        self.skins_done = 0
        self.gripper = RG6(self)
        self.gripper.set_gripper_pose(150, wait=True)

        self.toggle_gravity()

    def kill_open3d(self):
        # a bit of a workaround to kill open3d, that seems to hang for some reason
        for _ in os.popen("pgrep -f " + self.parent_name).read().strip().splitlines():
            call("kill -9 " + _, shell=True)

    def init_robot(self):
        """
        Load the robot URDF and get its joints' information

        :return: robot and its joints
        :rtype: int or list
        """
        if self.config.self_collisions:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True, flags=self.URDF_USE_SELF_COLLISION)
        else:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True)

        joints = []
        for joint in range(self.getNumJoints(robot)):
            info = self.getJointInfo(robot, joint)
            if info[self.jointInfo["TYPE"]] != p.JOINT_FIXED:
                joint = Joint(str(info[self.jointInfo["NAME"]], "utf-8"), info[self.jointInfo["INDEX"]], len(joints),
                              info[self.jointInfo["LOWERLIMIT"]], info[self.jointInfo["UPPERLIMIT"]],
                              info[self.jointInfo["MAXFORCE"]], info[self.jointInfo["MAXVELOCITY"]])
                joints.append(joint)
                if hasattr(self.config.initial_joint_angles, joint.name):
                    self.resetJointState(robot, joint.robot_joint_id, np.deg2rad(getattr(self.config.initial_joint_angles, joint.name)))

        links = []
        for link in self.urdfs["robot"].links:
            if hasattr(link, "collision"):
                link_id = self.find_link_id(os.path.basename(link.collision.geometry.mesh.filename), robot=robot)
                link = Link(link.name, link_id, link)
                links.append(link)

        # perform one step of collision detection
        self.stepSimulation()
        # get all collisions
        self_collisions = self.getContactPoints(robot, robot)
        # disable collision for all links in collision -> these links should be in collision by default, so we need
        # to disable checks for them
        for c in self_collisions:
            self.setCollisionFilterPair(robot, robot, c[3], c[4], False)

        if self.config.skin.use:
            for obj_id, obj_name, _, _, _ in self.other_objects:
                if obj_name == "airskin_ball":
                    for link in links:
                        for obj_link_id in range(len(self.urdfs[obj_name].links)):
                            self.setCollisionFilterPair(robot, obj_id, link.robot_link_id, obj_link_id, False)

        return robot, joints, links

    def is_alive(self):
        """
        Checks whether the engine is still running

        :return: True when running
        :rtype: bool
        """
        if self.gui and not self.visualizer.is_alive:
            return False
        return True if self._client >= 0 else False

    def update_simulation(self, sleep_duration=0.01):
        """
        Updates the simulation

        :param sleep_duration: duration to sleep before the next simulation step
        :type sleep_duration: float, optional, default=0.01
        """

        # This is here to keep events and everything in open3D work even if we want slower simulation
        if sleep_duration is None or time.time()-self.last_step > sleep_duration:
            self.stepSimulation()
            self.last_step = time.time()
            self.steps_done += 1

            # # This is not tested!
            info = self.getJointState(self.robot, self.gripper.finger_joint_id)
            if info[self.jointStates["TORQUE"]] > self.gripper.max_force:
                self.stop_robot(["finger_joint"])

            if self.config.log.log and time.time()-self.last_log > self.config.log.period:
                self.file_logger.info(self.prepare_log())
                self.last_log = time.time()

        if self.config.skin.use and time.time() - self.last_skin_step > self.config.skin.period:
            self.compute_skin()
            self.last_skin_step = time.time()
            self.skins_done += 1

        if self.gui and time.time()-self.last_render > 0.01 and self.visualizer.is_alive:
            self.visualizer.render()
            self.last_render = time.time()

    def toggle_gravity(self):
        """
        Toggles the gravity

        """
        if not self.gravity:
            self.gravity = True
            self.setGravity(0, 0, -9.81)
        else:
            self.gravity = False
            self.setGravity(0, 0, 0)

    def __del__(self):
        """
        Destructor to make sure the engine is closed

        """
        self.disconnect()

    def compute_skin(self):
        """
        Function to compute the skin activations

        :return:
        :rtype:
        """
        # Reset all skin activations
        for f_path in self.activated_skin:
            self.activated_skin[f_path] = 0
        # enumerate all other objects in the simulation
        for other_obj, _, _, _, _ in self.other_objects:
            # compute the closest points on the robot for given object
            close_points = self.getClosestPoints(self.robot, other_obj, 0.03)

            # number to string to robot link magic
            links_in_contact = {}
            for pair in close_points:
                link = pair[3]
                if link in self.link_to_skin:
                    if link in links_in_contact:
                        links_in_contact[link].append(pair[8])
                    else:
                        links_in_contact[link] = [pair[8]]
            # if contact detected, compute the distance and update the skin activation
            if len(links_in_contact) > 0:
                link_in_contact = sorted(list(links_in_contact.keys()), key=lambda x: np.min(links_in_contact[x]))[0]
                dists = []
                # get distances to all airskin links in possible collision
                for airskin_link in self.link_to_skin[link_in_contact][1]:
                    airskin_pos = self.getLinkState(self.robot, airskin_link, computeLinkVelocity=0, computeForwardKinematics=0)[self.linkInfo["WORLDPOS"]]
                    object_pos, _ = self.getBasePositionAndOrientation(other_obj)
                    dists.append(np.linalg.norm(np.array(airskin_pos) - np.array(object_pos)))

                for f_path in self.activated_skin:
                    f_path_comp = f_path.split("_vhacd")[0].split("/")[-1]
                    if self.link_to_skin_string[self.link_to_skin[link_in_contact][0]][np.argmin(dists)].split("__")[0] == f_path_comp:
                        dist = np.min(links_in_contact[link_in_contact])  # the minimal distance is the one we want
                        self.activated_skin[f_path] = (0.03-dist)*10000  # some magic to get a nice number
                        break

    def prepare_log(self):
        """
        Prepares the log string

        :return: log string
        :rtype: str
        """

        states = self.getJointStates(self.robot, [_.robot_joint_id for _ in self.joints])
        joint_states = ";".join([str(_[0]) for _ in states])

        s = f"{self.last_step};{self.steps_done};{joint_states}"

        if self.config.skin.use:
            s += ";"+";".join([str(self.activated_skin[_]) for _ in self.sorted_skin_links])
        return s

    def move_position(self, joints, positions, wait=True, velocity=None, set_col_state=True, check_collision=True):
        """
        Move the specified joints to the given positions

        :param joints: joint or list of joints to move
        :type joints: int, list, str
        :param positions: position or list of positions to move the joints to
        :type positions: float or list
        :param wait: whether to wait until the motion is done
        :type wait: bool, optional, default=True
        :param velocity: velocity to move the joints with
        :type velocity: float, optional, default=1
        :param set_col_state: whether to reset collision state
        :type set_col_state: bool, optional, default=True
        :param check_collision: whether to check for collision during motion
        :type check_collision: bool, optional, default=True
        """
        if not (isinstance(joints, list) or isinstance(joints, np.ndarray)):
            positions = [positions]
            joints = [joints]

        for joint, position in zip(joints, positions):

            robot_joint_id, joint_id = self.find_joint_id(joint)
            if not (self.joints[joint_id].lower_limit <= position <= self.joints[joint_id].upper_limit):
                self.logger.warning(f"Joint {joint} cannot be moved to {position} as it is out of bounds "
                                    f"({self.joints[joint_id].lower_limit}, {self.joints[joint_id].upper_limit}).")
                continue
            self.joints[joint_id].set_point = position
            if velocity is None:
                velocity = self.joints[joint_id].max_velocity
            self.setJointMotorControl2(self.robot, robot_joint_id,
                                       controlMode=self.POSITION_CONTROL, targetPosition=position,
                                       force=self.joints[joint_id].max_force,
                                       maxVelocity=velocity)
        if set_col_state:
            self.collision_during_motion = False
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def move_velocity(self, joints, velocities):
        """
        Move the specified joints with the specified velocity

        :param joints: joint or list of joints to move
        :type joints: int or list
        :param velocities: velocity or list of velocities to move the joints to
        :type velocities: float or list
        """
        if not (isinstance(joints, list) or isinstance(joints, np.ndarray)):
            velocities = [velocities]
            joints = [joints]

        for joint, velocity in zip(joints, velocities):

            robot_joint_id, joint_id = self.find_joint_id(joint)
            if np.abs(velocity) > self.joints[joint_id].max_velocity:
                self.logger.warning(f"Joint {joint} cannot be moved with velocity {velocity} as it is over the max velocity "
                                    f"{self.joints[joint_id].max_velocity}")
                continue
            self.setJointMotorControl2(self.robot, robot_joint_id,
                                       controlMode=self.VELOCITY_CONTROL, targetVelocity=velocity,
                                       force=self.joints[joint_id].max_force,
                                       maxVelocity=self.joints[joint_id].max_velocity)

    def get_joint_state(self, joints=None):
        """
        Get the state of the specified joints

        :param joints: joint or list of joints to get the state of
        :type joints: int or list, optional, default=None
        :return: list of states of the joints
        :rtype: list
        """
        if joints is None:
            joints = [joint.name for joint in self.joints]
        elif not isinstance(joints, list):
            joints = [joints]

        states = []
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            states.append(self.getJointState(self.robot, robot_joint_id)[self.jointStates["POSITION"]])

        return states

    def motion_done(self, joints=None, check_collision=True):
        """
        Checks whether the motion is done.

        :param joints: joint or list of joints to get the state of
        :type joints: int or list, optional, default=None
        :param check_collision: whether to check for collision during motion
        :type check_collision: bool, optional, default=True
        :return: True when motion is done, false otherwise
        :rtype: bool
        """
        if joints is None:
            joints = [joint.name for joint in self.joints]
        elif not isinstance(joints, list):
            joints = [joints]

        if check_collision:
            contacts = self.getContactPoints(self.robot)
            for c in contacts:
                if c[self.contactPoints["IDB"]] not in self.free_objects and c[self.contactPoints["DISTANCE"]] < self.config.collision_tolerance:
                    self.collision_during_motion = True
                    self.stop_robot()
                    self.logger.warning("Collision detected during motion!")
                    self.print_collision_info()
                    return True
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None:
                if np.abs(state[self.jointStates["POSITION"]] - self.joints[joint_id].set_point) > self.joint_tolerance:
                    return False

        self.stop_robot()
        return True

    def wait_motion_done(self, sleep_duration=0.01, check_collision=True):
        """
        Help function to wait for motion to be done. Can sleep for a specific duration

        :param sleep_duration: how long to sleep before running simulation step
        :type sleep_duration: float, optional, default=0.01
        :param check_collision: whether to check for collisions during motion
        :type check_collision: bool, optional, default=True
        """
        while not self.motion_done(check_collision=check_collision):
            self.update_simulation(sleep_duration)

    def stop_robot(self, joints=None):
        """
        Stops the robot

        """
        if joints is None:
            joints = [joint.name for joint in self.joints]
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None:
                self.move_position(joint, state[self.jointStates["POSITION"]], wait=False, set_col_state=False)
                self.joints[joint_id].set_point = None

    def move_cartesian(self, pose, wait=True, velocity=1, check_collision=True):
        """
        Move the robot in cartesian space by computing inverse kinematics and running position control

        :param pose: desired pose of the end effector
        :type pose: utils.Pose
        :param wait: whether to wait for movement completion
        :type wait: bool, optional, default=True
        :param velocity: joint velocity to move with
        :type velocity: float, optional, default=1
        :param check_collision: whether to check for collisions during motion
        :type check_collision: bool, optional, default=True
        """
        ik_solution = np.array(self.calculateInverseKinematics(self.robot, self.end_effector.link_id, pose.pos, pose.ori,
                                                               lowerLimits=self.IK_config["lower_limits"],
                                                               upperLimits=self.IK_config["upper_limits"],
                                                               jointRanges=self.IK_config["joint_ranges"],
                                                               restPoses=self.IK_config["rest_poses"]))
        self.move_position(self.IK_config["movable_joints"], ik_solution[self.IK_config["movable_joints"]], wait=False,
                           velocity=velocity)
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def find_joint_id(self, joint_name):
        """
        Help function to get indexes from joint name of joint index in self.joints list

        :param joint_name: name or index of the link
        :type joint_name: str or int
        :return: joint id in pybullet and pycub space
        :rtype: int, int
        """
        for joint in self.joints:
            if joint_name in [joint.name, joint.joints_id]:
                return joint.robot_joint_id, joint.joints_id

    def find_link_id(self, mesh_name, robot=None, urdf_name="robot"):
        """
        Help function to find link id from mesh name

        :param mesh_name: name of the mesh (only basename with extension)
        :type mesh_name: str
        :param robot: robot pybullet id
        :type robot: int, optional, default=None
        :param urdf_name: name of the object in pycub.urdfs
        :type urdf_name: str, optional, default="robot"
        :return: id of the link in pybullet space
        :rtype: int
        """
        if robot is None:
            robot = self.robot

        for link_id in range(0, len(self.urdfs[urdf_name].links)-1):
            cs = self.getCollisionShapeData(robot, link_id)
            if len(cs) > 0:
                if mesh_name == os.path.basename(cs[0][4].decode("utf-8")):
                    return link_id

    def get_link_id(self, link_name):
        for link in self.links:
            if link_name == link.name:
                return link.robot_link_id

    def run_vhacd(self):
        """
        Function to run VHACD on all objects in loaded URDFs, and to create new URDFs with changed collision meshes

        """
        something_changed = False
        for obj_name, obj in self.urdfs.items():
            for link in obj.links:
                if hasattr(link, "collision"):
                    if hasattr(link.collision.geometry, "mesh"):
                        col_path_ori = link.collision.geometry.mesh.filename
                        col_path = col_path_ori.replace("package://", "")
                        col_path = os.path.normpath(os.path.join(self.file_dir, "../../../", col_path))
                        vhacd_path = col_path.replace(".obj", "_vhacd.obj").replace("visual", "collision")
                        if self.config.vhacd.force_vhacd or not os.path.exists(vhacd_path):
                            self.vhacd(col_path, vhacd_path, "", resolution=1000000, maxNumVerticesPerCH=1, gamma=0.0005, concavity=0)
                        if self.config.vhacd.force_vhacd_urdf or not os.path.exists(vhacd_path):
                            something_changed = True
                        link.collision.geometry.mesh.filename = col_path_ori.replace("visual", "vhacd").replace(".obj", "_vhacd.obj")
            obj.path = obj.path.replace("_fixed", "").replace(".urdf", "_vhacd.urdf")
            if something_changed or self.config.vhacd.force_vhacd_urdf or not os.path.exists(obj.path):
                obj.write_urdf()

                with open(obj.path, "w") as f:
                    f.write(obj.new_urdf)

    def create_urdf(self, object_path, fixed, color, suffix=""):
        """
        Creates a URDF for the given .obj file

        :param object_path: path to the .obj
        :type object_path: str
        :param fixed: whether the object is fixed in space
        :type fixed: bool
        :param color: color of the object
        :type color: list of 3 floats

        """
        with open(os.path.join(self.file_dir, "../../..", "other_meshes", "object_default.urdf"), "r") as f:
            urdf = f.read()
        if suffix != "":
            mesh = o3d.io.read_triangle_mesh(os.path.normpath(os.path.join(self.file_dir, "../../../other_meshes", object_path)))
        object_path = object_path.replace(".obj", suffix+".obj")
        object_path = os.path.normpath(os.path.join(self.file_dir, "../../../other_meshes", object_path))
        if suffix != "":
            o3d.io.write_triangle_mesh(object_path, mesh)

        model_name = os.path.basename(object_path).split(".")[0]
        urdf = urdf.replace("OBJECTNAME", model_name).replace("LATERALFRICTION", "1") \
            .replace("ROLLINGFRICTION", "0").replace("MASS", "0.1").replace("FILENAMECOLLISION", object_path) \
            .replace("FILENAME", object_path).replace("VISUALCOLOR", " ".join(map(str, color)))

        if fixed:
            with open(os.path.join(self.file_dir, "../../../", "other_meshes", "fixed_link.txt"), "r") as f:
                fixed_link_text = f.read()
            urdf = urdf.replace("</robot>", fixed_link_text)

        with open(object_path.replace(".obj", ".urdf"), "w") as f:
            f.write(urdf)

    def print_collision_info(self, c=None):
        """
        Help function to print collision info

        :param c: one collision
        :type c: list, optional, default=None
        """
        if c is None:
            contacts = self.getContactPoints(self.robot)
            for c in contacts:
                self.print_collision_info(c)
        else:
            if c[self.contactPoints['IDB']] == self.robot:
                body_b = "robot"
            else:
                for obj, name, _, _, _ in self.other_objects:
                    if c[self.contactPoints['IDB']] == obj:
                        body_b = name
                        break
            for link in self.links:
                if link.robot_link_id == c[self.contactPoints['INDEXA']]:
                    break
            link_a = link.name
            if c[self.contactPoints['IDB']] == self.robot:
                for link in self.links:
                    if link.robot_link_id == c[self.contactPoints['INDEXB']]:
                        break
                link_b = link.name
            else:
                link_b = f"{body_b} link {c[self.contactPoints['INDEXB']]}"
            self.logger.info(f"\nCollision of robot with {body_b}\n"
                             f"Collision of {link_a} with {link_b}\n"
                             f"Position A: {c[self.contactPoints['POSITIONA']]}\n"
                             f"Position B: {c[self.contactPoints['POSITIONB']]}\n"
                             f"Normal: {c[self.contactPoints['NORMAL']]}\n"
                             f"Distance: {c[self.contactPoints['DISTANCE']]}\n"
                             f"Force: {c[self.contactPoints['FORCE']]}\n"
                             f"Friction 1: {c[self.contactPoints['FRICTION1']]}\n"
                             f"Friction dir 1: {c[self.contactPoints['FRICTIONDIR1']]}\n"
                             f"Friction 2: {c[self.contactPoints['FRICTION2']]}\n"
                             f"Friction dir 2: {c[self.contactPoints['FRICTIONDIR2']]}\n")


class Joint:
    def __init__(self, name, robot_joint_id, joints_id, lower_limit, upper_limit, max_force, max_velocity):
        """
        Help class to encapsulate joint information

        :param name: name of the joint
        :type name: str
        :param robot_joint_id: id of the joint in pybullet
        :type robot_joint_id: int
        :param joints_id: id of the joint in pycub.joints
        :type joints_id: int
        :param lower_limit: lower limit of the joint
        :type lower_limit: float
        :param upper_limit: upper limit of the joint
        :type upper_limit: float
        :param max_force: max force of the joint
        :type max_force: float
        :param max_velocity: max velocity of the joint
        :type max_velocity: float
        """
        self.name = name
        self.robot_joint_id = robot_joint_id
        self.joints_id = joints_id
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.max_force = max_force
        self.max_velocity = max_velocity
        self.set_point = None

    def __repr__(self):
        return f"Joint {self.name} with id {self.robot_joint_id}"


class Link:
    def __init__(self, name, robot_link_id, urdf_link):
        """
        Help function to encapsulate link information

        :param name: name of the link
        :type name: str
        :param robot_link_id: id of the link in pybullet
        :type robot_link_id: int
        :param urdf_link: id of the link in pycub.urdfs["robot"].links
        :type urdf_link: int
        """
        self.name = name
        self.robot_link_id = robot_link_id
        self.urdf_link = urdf_link


class EndEffector:
    def __init__(self, name, client):
        """
        Help function for end-effector encapsulaation

        :param name: name of the end-effector
        :type name: str
        :param client: parent client
        :type client: pointer to pyCub instance
        """
        self.name = name
        self.client = client
        for link in self.client.urdfs["robot"].links:
            if link.name == self.name:
                self.link_id = self.client.find_link_id(os.path.basename(link.collision.geometry.mesh.filename))
                break

    def get_position(self):
        """
        Function to get current position of the end-effector
        """
        state = self.client.getLinkState(self.client.robot, self.link_id)
        pos = list(state[self.client.linkInfo["URDFPOS"]])
        ori = list(state[self.client.linkInfo["URDFORI"]])
        return Pose(pos, ori)

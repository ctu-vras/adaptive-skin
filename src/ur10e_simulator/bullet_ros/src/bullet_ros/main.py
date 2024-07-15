#!/usr/bin/env python3
import rospy
from pyUR.pyur import pyUR
from bullet_ros.msg import bullet_to_ros
from bullet_ros.msg import ros_to_bullet
import numpy as np
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from bullet_ros.srv import grip, gripResponse, changeObjPose, changeObjPoseResponse, activateAirskin, activateAirskinResponse
from bullet_ros.msg import AirskinStatus
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion


class BulletROS:
    def __init__(self):
        rospy.init_node("pyUR")
        self.client = pyUR(config="ros.yaml")

        # UR10e joint names in the correct order
        joints = "shoulder_pan_joint;shoulder_lift_joint;elbow_joint;wrist_1_joint;wrist_2_joint;wrist_3_joint"

        # Get the joint ids of the UR10e joints
        self.joint_names = joints.split(";")
        self.joint_ids = []
        for joint in self.joint_names:
            self.joint_ids.append(self.client.find_joint_id(joint)[0])
        self.joint_names = np.array(self.joint_names)

        # Initialize the message to be published
        self.msg = bullet_to_ros()
        self.msg.positions = [0] * len(self.joint_ids)
        self.msg.velocities = [0] * len(self.joint_ids)
        self.msg.efforts = [0] * len(self.joint_ids)

        # Initialize the grip service
        self.grip_service = rospy.Service("/robot/ur_hardware_interface/grip", grip, self.grip)
        self.MAX_WIDTH = 150
        self.MAX_FORCE = 120

        # Initialize the initial joint angles
        init_pos = []
        for j_name in self.joint_names:
            for joint in self.client.joints:
                if j_name == joint.name:
                    if hasattr(self.client.config.initial_joint_angles, joint.name):
                        init_pos.append(np.deg2rad(getattr(self.client.config.initial_joint_angles, joint.name)))
                    else:
                        init_pos.append(0)
                    break

        # Set the joint names and initial joint angles as parameters
        rospy.set_param("/bullet_ros/joint_names", joints)
        rospy.set_param("/bullet_ros/init_pos", ";".join(map(str, init_pos)))
        self.cmd = None
        self.move_type = 0  # 0 - position, 1 - velocity

        # Initialize the finger joint state
        self.finger_js = JointState()
        self.finger_js.header = Header()
        self.finger_js.name = ["finger_joint"]

        # Initialize the joint state publisher for finger joint
        self.js_pub = rospy.Publisher("/finger/joint_states", JointState, queue_size=1)

        # Initialize the skin publisher
        self.skin_pub = rospy.Publisher("/airskin_status", AirskinStatus, queue_size=1)
        self.airskin_msg = AirskinStatus()
        self.airskin_msg.pressures = [0] * 11

        # Initialize the marker publisher for balls in RVIZ
        self.marker_array = MarkerArray()
        self.marker_pub = rospy.Publisher("/other_objects", MarkerArray, queue_size=1)

        # Initialize the publisher for the bullet to ros message
        self.pub = rospy.Publisher("/bullet_ros/read", bullet_to_ros, queue_size=1)

        # Initialize the controller service
        self.controller_srv = rospy.ServiceProxy("/robot/controller_manager/list_controllers", ListControllers)

        # Initialize the service to change the object pose
        self.free_obj_service = rospy.Service("/bullet_ros/change_obj_pose", changeObjPose, self.change_obj_pose)
        self.pad_to_activate = None
        if self.client.config.skin.use:
            self.activate_airskin_service = rospy.Service("/bullet_ros/activate_airskin", activateAirskin, self.activate_airskin_cb)
            self.airskin_ball_id = None
            self.airskin_pad_id = None
            self.airskin_effort = None
            self.airskin_effort_init = None
            self.airskin_ball_pos = None
            self.follow_pad = None
            self.increase_effort = None

        # Initialize other variables
        self.fixed_states = None
        self.last_pos_cmd = None
        self.last_skin_time = rospy.get_time()

        # Lister to ROS RobotHW interface
        self.sub = rospy.Subscriber("/bullet_ros/write", ros_to_bullet, self.read, queue_size=1)

    def change_obj_pose(self, request):
        """
        Change pose of other than robot object

        :param request: request fot the pose change
        :type request: bullet_ros.srv.changeObjPoseRequest
        :return:
        :rtype:
        """
        # Check if the object exists
        found = False
        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            if obj_name == request.name.data:
                found = True
                break
        if not found:
            rospy.logerr(f"Object {request.name.data} not found")
            return changeObjPoseResponse(0)

        # Prepare pose based on request
        if len(request.position) != 3:
            pos = [0, 0, 0]
        else:
            pos = request.position
        if len(request.orientation) != 4:
            ori = [0, 0, 0, 1]
        else:
            ori = request.orientation

        # Call bullet to change the pose
        self.client.resetBasePositionAndOrientation(obj_id, pos, ori)
        return changeObjPoseResponse(1)

    def activate_airskin_cb(self, request):
        """
        Callback for airskin activation request

        :param request: request with all the info
        :type request: bullet_ros.srv.activateAirskinRequest
        :return:
        :rtype:
        """
        if not (0 <= request.effort <= 1):
            rospy.logerr("Effort must be between 0-1")
            return activateAirskinResponse(0)

        # Change 0-1 to -1 - 1 because of some magic
        self.airskin_effort = ((request.effort - 0) / (1 - 0)) * (1 - -1) + -1
        self.airskin_effort_init = self.airskin_effort
        self.pad_to_activate = request.pad_id

        # if any pad to activate
        if request.pad_id != -1:
            # find ball id (need to be done just for the first time)
            if self.airskin_ball_id is None:
                for obj_id, obj_name, _, _, _ in self.client.other_objects:
                    if obj_name == "airskin_ball":
                        break
                self.airskin_ball_id = obj_id
            # find pad id
            for link in self.client.links:
                if "airskin" in link.name and "__" not in link.name and int(link.name.split("_")[-1]) == request.pad_id:
                    #if self.airskin_pad_id is not None and self.airskin_pad_id != link.robot_link_id:
                    self.airskin_ball_pos = None
                    self.airskin_pad_id = link.robot_link_id
                    break
            self.follow_pad = request.follow
            self.increase_effort = request.increase_effort
        return activateAirskinResponse(1)

    def activate_airskin(self):
        """
        Function to activate the airskin based on variables that had been set through service

        :return:
        :rtype:
        """
        # if current activation
        if self.pad_to_activate is None:
            return 0
        # deaction of the activation
        elif self.pad_to_activate == -1:
            self.pad_to_activate = None
            self.airskin_pad_id = None
            self.airskin_effort = None
            self.airskin_effort_init = None
            self.airskin_ball_pos = None
            self.increase_effort = None
            # This is just to put the ball out of view of the user
            self.client.resetBasePositionAndOrientation(self.airskin_ball_id, [0, 0, -1], [0, 0, 0, 1])
            return 0
        # Compute new ball position
        if (self.follow_pad or self.airskin_ball_pos is None or
                (self.increase_effort and self.airskin_effort < (self.airskin_effort_init+0.25) and self.airskin_effort < 1)):

            linkState = self.client.getLinkState(self.client.robot, self.airskin_pad_id, computeLinkVelocity=1, computeForwardKinematics=0)
            pos = np.array(linkState[self.client.linkInfo["WORLDPOS"]])
            ori = np.array(linkState[self.client.linkInfo["WORLDORI"]])
            R = np.eye(4)
            R[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(ori), (3, 3))
            u = np.matmul(R, np.hstack(([0, 0, 1], 1)))[:3]  # go against normal of the given link
            u /= np.linalg.norm(u)
            u *= (1-self.airskin_effort)*0.03
            pos += u
            self.airskin_ball_pos = pos
            if self.increase_effort:
                self.airskin_effort += 0.02
                self.airskin_effort = np.min([1, self.airskin_effort])
        # Set the ball position (either new or use old one)
        self.client.resetBasePositionAndOrientation(self.airskin_ball_id, self.airskin_ball_pos, [0, 0, 0, 1])

    def read(self, msg):
        """
        Function to read the message from the RobotHW interface and send it to the bullet

        :param msg: msg with velocity and joint angles
        :type msg: bullet_ros.msg.ros_to_bullet
        :return:
        :rtype:
        """
        self.cmd = msg

        # Check which controller is running and based on that select how to move the robot (velocity or position)
        controller_info = self.controller_srv.call()
        for c in controller_info.controller:
            if c.name == "joint_group_vel_controller":
                if c.state == "running":
                    if self.move_type == 0:
                        self.last_pos_cmd = self.cmd.cmd
                    self.move_type = 1
                else:
                    if self.move_type == 1:
                        if self.last_pos_cmd is not None and np.allclose(self.last_pos_cmd, self.cmd.cmd):
                            self.move_type = 1
                        else:
                            self.last_pos_cmd = None
                            self.move_type = 0
                    else:
                        self.move_type = 0
                break

        # Main loop - > call bullet movement function, update simulation, read new state, activate airskin and send all
        self.move()
        self.client.update_simulation(None)
        self.read_state()
        self.send_finger_joint()
        self.activate_airskin()
        if self.client.config.skin.use and rospy.get_time() - self.last_skin_time > self.client.config.skin.period:
            self.send_skin()
            self.last_skin_time = rospy.get_time()
        self.publish_other_objects()
        self.pub.publish(self.msg)

    def publish_other_objects(self):
        """
        Function to publish other than robot objects to RVIZ

        :return:
        :rtype:
        """
        marker_id = 0
        for obj_id, obj_name, is_fixed, color, mesh in self.client.other_objects:
            pos, ori = self.client.getBasePositionAndOrientation(obj_id)
            if len(self.marker_array.markers) < len(self.client.other_objects):
                marker = Marker()
                marker.id = marker_id
                marker_id += 1
                marker.ns = 'mesh_marker'
                marker.header.frame_id = "base_link"
                marker.action = marker.ADD
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1
                marker.color.a = 1.0
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.type = marker.MESH_RESOURCE
                marker.mesh_resource = "package://bullet_ros/../other_meshes/" + mesh
                marker.pose.position = Point(*pos)
                marker.pose.orientation = Quaternion(*ori)
                self.marker_array.markers.append(marker)
            else:
                self.marker_array.markers[marker_id].action = Marker.MODIFY
                self.marker_array.markers[marker_id].pose.position = Point(*pos)
                self.marker_array.markers[marker_id].pose.orientation = Quaternion(*ori)
                marker_id += 1
        self.marker_pub.publish(self.marker_array)

    def send_skin(self):
        """
        Function to send current airskin values to /airskin_status topic

        :return:
        :rtype:
        """
        noise = np.random.uniform(-1, 1, 11)
        for pad in range(11):
            self.airskin_msg.pressures[pad] = self.client.activated_skin[self.client.sorted_skin_links[pad]] + noise[pad]
        self.skin_pub.publish(self.airskin_msg)

    def read_state(self):
        """
        Read current robot state to be sent back to RobotGH interface

        :return:
        :rtype:
        """
        state = self.client.getJointStates(self.client.robot, self.joint_ids)
        for idx, j_state in enumerate(state):
            self.msg.positions[idx] = j_state[0]
            self.msg.velocities[idx] = j_state[1]
            self.msg.efforts[idx] = j_state[3]

    def send_finger_joint(self):
        """
        Get state of finger joint and publish it

        :return:
        :rtype:
        """
        state = self.client.getJointState(self.client.robot, self.client.gripper.finger_joint_id)
        self.finger_js.header.stamp = rospy.Time.now()
        self.finger_js.position = [state[0]]
        self.finger_js.velocity = [state[1]]
        self.finger_js.effort = [state[3]]
        self.js_pub.publish(self.finger_js)

    def move(self):
        """
        Based on move_type (velocity or position) move the robot in the given pose or fix it in place

        :return:
        :rtype:
        """
        if self.cmd is None:
            return 0
        if self.move_type == 0:  # Position = just send directly
            self.client.move_position(self.joint_names, self.cmd.cmd, wait=False)

        elif self.move_type == 1:
            # if all velocity command are not 0 send them to the robot
            if not np.allclose(self.cmd.vel_cmd, np.zeros(len(self.cmd.vel_cmd))):
                self.fixed_states = None
                self.client.move_velocity(self.joint_names, np.array(self.cmd.vel_cmd))
            else: # all velocities 0
                if self.fixed_states is None:  # we do not know where to keep the robot
                    state = self.client.getJointStates(self.client.robot, self.joint_ids)
                    pos = np.zeros(len(self.joint_ids))
                    for idx, j_state in enumerate(state):
                        pos[idx] = j_state[0]
                    self.fixed_states = pos
                # Keep the robot on place
                self.client.move_position(self.joint_names, self.fixed_states, wait=False)

    def grip(self, request):
        """
        Grip request callback

        :param request: request to close/open the gripper
        :type request: bullet_ros/srv/gripRequest
        :return:
        :rtype:
        """
        if not (0 <= request.width <= self.MAX_WIDTH):
            rospy.logerr("Width must be between 0-150")
            out = gripResponse()
            out.success = False
            return out

        if not (0 < request.force <= self.MAX_FORCE):
            rospy.logerr("Force must be between 0-120")
            out = gripResponse()
            out.success = False
            return out

        vel_min = 0
        vel_max = 2

        # Calculate the velocity based on the force
        velocity = ((request.force - 0) / (120 - 0)) * (vel_max - vel_min) + vel_min

        self.client.gripper.set_gripper_pose(request.width, velocity=velocity, wait=False)  # This needs to be fixed
        self.client.gripper.max_force = request.force

        # wait until the gripper is closed
        while not rospy.is_shutdown():
            info = self.client.getJointState(self.client.robot, self.client.gripper.finger_joint_id)
            if np.abs(info[self.client.jointStates["POSITION"]] - self.client.gripper.joint_ref.set_point) <= self.client.joint_tolerance:
                self.client.stop_robot(["finger_joint"])
                break
            rospy.sleep(0.05)

        out = gripResponse()
        out.success = True
        return out


if __name__ == "__main__":
    c = BulletROS()
    rospy.spin()

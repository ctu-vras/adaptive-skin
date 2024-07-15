#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <scaled_joint_trajectory_controller/scaled_joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <bullet_ros/bullet_to_ros.h>
#include <bullet_ros/ros_to_bullet.h>

class BulletRobot : public hardware_interface::RobotHW {
public:

    explicit BulletRobot();

    void read();

    void write();

    bullet_ros::bullet_to_ros pybullet_data;

    ros::Publisher pub;

    double scaling_factor;

    std::string joint_names_param_name = "/bullet_ros/joint_names";
    std::string init_pos_param_name = "/bullet_ros/init_pos";

public:

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    scaled_controllers::ScaledPositionJointInterface spj_interface;
    hardware_interface::VelocityJointInterface vj_interface;

    std::vector<double> vel_cmd, cmd, pos, vel, eff;

};
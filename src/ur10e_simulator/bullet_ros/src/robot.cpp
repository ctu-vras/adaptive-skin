#include <bullet_ros/robot.h>


BulletRobot::BulletRobot() {
    // Wait for joint names and split them by ';'
    std::string joint_names_param;
    while (ros::ok())
    {
        if (ros::param::get(joint_names_param_name, joint_names_param))
        {
            if (joint_names_param != "")
            {
                break;
            }
        }
    }

    std::vector<std::string> joint_names;
    std::string segment;
    std::stringstream joint_names_stream (joint_names_param);
    while(std::getline(joint_names_stream, segment, ';'))
    {
        joint_names.push_back(segment);
    }

    // wait for initial position and split them by ';'
    std::string init_pos_param;
    while (ros::ok())
    {
        if (ros::param::get(init_pos_param_name, init_pos_param))
            {
                if (init_pos_param != "")
                {
                    break;
                }
        }
    }

    std::vector<std::string> init_pos;
    std::stringstream init_pos_stream (init_pos_param);
    while(std::getline(init_pos_stream, segment, ';'))
    {
        init_pos.push_back(segment);
    }

    // Initialize joint state interface
    const auto n = joint_names.size();
    cmd.resize(n, 0.0);
    vel_cmd.resize(n, 0.0);
    pos.resize(n, 0.0);
    vel.resize(n, 0.0);
    eff.resize(n, 0.0);


    scaling_factor = 1.0;

    // create handles for each joint and put them to the joint state interface
    for (int j = 0; j < n; j++)
    {
        hardware_interface::JointStateHandle state_handle_a(joint_names[j], &pos[j], &vel[j], &eff[j]);
        jnt_state_interface.registerHandle(state_handle_a);

        scaled_controllers::ScaledJointHandle spj_handle_a(jnt_state_interface.getHandle(joint_names[j]), &cmd[j], &scaling_factor);
        spj_interface.registerHandle(spj_handle_a);

        hardware_interface::JointHandle vj_handle_a(jnt_state_interface.getHandle(joint_names[j]), &vel_cmd[j]);
        vj_interface.registerHandle(vj_handle_a);

        cmd[j] = std::stod(init_pos[j]);
        pos[j] = std::stod(init_pos[j]);
        std::cout << "Joint " << joint_names[j] << " initialized to " << cmd[j] << std::endl;
    }


    // register interfaces
    registerInterface(&jnt_state_interface);
    registerInterface(&spj_interface);
    registerInterface(&vj_interface);
}

void BulletRobot::read() {
    // read values from pybullet to interfaces
    pos = pybullet_data.positions;
    vel = pybullet_data.velocities;
    eff = pybullet_data.efforts;
}

void BulletRobot::write() {
    // send command to the robot
    bullet_ros::ros_to_bullet msg;
    msg.cmd = cmd;
    msg.vel_cmd = vel_cmd;
    pub.publish(msg);

}

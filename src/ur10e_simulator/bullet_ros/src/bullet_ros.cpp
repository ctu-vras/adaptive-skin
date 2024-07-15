#include <ros/ros.h>
#include <bullet_ros/robot.h>
#include <bullet_ros/bullet_to_ros.h>
#include <bullet_ros/ros_to_bullet.h>
#include <controller_manager/controller_manager.h>
std::unique_ptr<BulletRobot> hw;

void bullet_subscriber(const bullet_ros::bullet_to_ros::ConstPtr& msg) {
    hw->pybullet_data = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bullet_ros");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // prepare hardware interface and controller manager
    std::unique_ptr<controller_manager::ControllerManager> cm;

    hw.reset(new BulletRobot());
    cm.reset(new controller_manager::ControllerManager(hw.get(), n));

    // get subscriber/publisher
    ros::Subscriber sub = n.subscribe("/bullet_ros/read", 1, bullet_subscriber);
    hw->pub = n.advertise<bullet_ros::ros_to_bullet>("/bullet_ros/write", 1);


    // run
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        ros::Duration dt = now - last_time;

        // 0.004 = 250Hz control loop
        if (dt.toSec() < 0.004) {
            continue;
        }

        hw->read();
        cm->update(now, dt, false);
        hw->write();

        last_time = now;
    }
}
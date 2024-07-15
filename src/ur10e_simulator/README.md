# UR10e python simulator with virtual AIRSKIN

This folder contains necessary files to run the simulation of the UR10e robot with virtual AIRSKIN.

## Contents

 - [Installation](#installation)
 - [Code](#code)
 - [Run](#run)

## Installation
The installation is the same as for the [adaptive_skin](https://github.com/ctu-vras/adaptive-skin) project. Please see 
the README there.

## Code
The documentation PDF can be found in [ur10ewithairskin.pdf](documentation/ur10ewithairskin.pdf).  
Online documentation is available at [lukasrustler.cz/pyur](https://lukasrustler.cz/pyur/).


## Run
The simulator can be run in two mode:
  1) ROS version - real-time, high-level planners (MoveIt!, ...)
  2) Native - much faster than real-time, only low-level control 
### ROS version
  - install necessary things and build the workspace
    - see [Installation](#Installation)
  - run the simulator
    - in the terminal, run

          roslaunch bullet_ros simulation.launch
    - this will start the simulator and RVIZ with the robot
#### Control the robot
 - to test the connection etc. you can just move the robot in RVIZ and plan
   from there
 - or, see [examples.py](src/ur10e_simulator/bullet_ros/scripts/examples.py)
   - it shows how to control the robot, gripper. How to play trajectories and how
     to get IK without moving
 - the simulated robot provides `position_controllers/ScaledJointTrajectoryController` and 
   `velocity_controllers/JointGroupVelocityController`, i.e., waypoint controller through moveit or direct assignment
   of joint velocities to the joints
   - the default is `ScaledJointTrajectoryController`. You need to switch with rosservice `/controller_manager/switch_controller`
     before running joint commands
     - see [examples.py](src/ur10e_simulator/bullet_ros/scripts/examples.py) for an example

### Non-ROS version
  - install necessary things
    - see [Installation](#Installation)
  - you can run examples from [examples](src/ur10e_simulator/pyUR/examples) folder
    - scripts [cartesian_example.py](src/ur10e_simulator/pyUR/examples/cartesian_example.py) and
      [joints_example.py](src/ur10e_simulator/pyUR/examples/joints_example.py) show how to control the robot in 
      cartesian and joint space, respectively.
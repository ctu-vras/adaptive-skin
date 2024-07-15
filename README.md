# Adaptive Electronic Skin Sensitivity for Safe Human-Robot Interaction

This repository contains code for paper "Adaptive Electronic Skin Sensitivity for Safe Human-Robot Interaction" by 
Lukáš Rustler, Matěj Mísař and Matěj Hoffmann.  

**Maintainer**: Lukas Rustler, lukas.rustler@fel.cvut.cz  

# Contents

 - [Licensing](#licensing)
 - [Installation](#installation)
 - [Docker Installation](#docker-installation)
 - [Code](#code)
 - [Run](#run)
 - [Data](#data)
 - [Real Setup](#real-setup)

## Licensing
The code created in this work is under GNU-3 licence.  
For licensing information about other used repositories see: 
[LICENSE](LICENSE), and the original repositories:  
- [kdl_parser_py](https://github.com/ros/kdl_parser) 
- [orocos](https://github.com/orocos/orocos_kinematics_dynamics)


# Installation
  - clone this repository
    
        cd SOME_PATH
        git clone https://github.com/ctu-vras/adaptive-skin.git adaptive_skin_ws

  - use Docker (see [Docker Installation](#docker-installation) or install ROS noetic (+ all necessary libraries;
    see [Dockerfile](Docker/Dockerfile)) and python3.8 (with pybullet, open3d and their dependencies)
  - if using Docker

        cd adaptive_skin_ws/Docker
        `./deploy.py -c adaptive_skin -p SOME_PATH/adaptive_skin_ws -b`
    - the above will build the Docker image, rename the container to 'adaptive_skin' and run it. You can also:
      - `./deploy.py -c adaptive_skin -e` to run already built container
      - `./deploy.py -c adaptive_skin -t` to open new terminal in the container
      - see [easy-docker](https://github.com/rustlluk/easy-docker) for more _deploy.py_ options
  - build the workspace

        cd adaptive_skin_ws
        catkin config --extend /opt/ros/noetic --init
        catkin build 
  - source the workspace

        source devel/setup.bash

# Docker Installation
  - **Works only on GNU/Linux systems!**
  - install [docker-engine](https://docs.docker.com/engine/install/ubuntu/)
  - **DO NOT INSTALL DOCKER-DESKTOP!**
    - docker-desktop is not the same as docker-engine, and it does not work the same way 
  - do not forget about [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
  - (optional) install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
    for GPU support

For more information, more commands and FAQ see [easy-docker](https://github.com/rustlluk/easy-docker)

# Code  

This section will talk mainly about the experiment code. If you want to learn more specifically only about the 
simulation, please look into README in [ur10e_simulator](src/ur10e_simulator).

The main package with experiment code is [airskin_pain](src/airskin_pain) with the following files:
  - [main.py](src/airskin-pain/src/airskin_pain/main.py) - main file that contains the task
  - [tester.py](src/airskin-pain/src/airskin_pain/tester.py) - script that reads config files and activates the skin (simulation only)
  - [utils.py](src/airskin-pain/src/airskin_pain/utils.py) - file with utility files (dynamics computation, airskin processing, etc.)
  - [do_exps.py](src/airskin-pain/src/airskin_pain/do_exps.py) - script that runs all necessary script to run the experiment
  - [airskin_node.py](src/airskin-pain/src/airskin_pain/airskin_node.py) - ROS node that reads the skin data and publishes touch events
  - [eff_mass.py](src/airskin-pain/src/airskin_pain/eff_mass.py) - script that computes effective mass of the robot and publishes thresholds

# Run

  1) The easiest way how to run your own experiments is to use [do_exps.py](src/airskin-pain/src/airskin_pain/do_exps.py)
    - run `rosrun airskin_pain do_exps.py -s SETUP -c CONFIGS -e EXP_NAME -i ITERATIONS`, where
      - _SETUP_ can "real" or "sim"
        - default: "sim"
      - _CONFIGS_ is the path to the config file/directory relative to [configs](src/airskin-pain/data/configs) directory
      - _EXP_NAME_ is the name of the experiment. This will be used for folder name in [exps](src/airskin-pain/data/exps) directory
        - you can omit this argument and the script will name the folder with current timestamp
      - _ITERATIONS_ is the number of iterations to run the experiment for each config
        - default: 10 
    - you can further specify in the command:
      - `--save_bag` to also save .bag file with all the data
        - default: not saving
      - `--offscreen` to run the experiments offscreen
        - default: RVIZ is shown with visualization  

  2) you can run everything by hand. Use the following commands and check parameters of each script in the corresponding file
    - run `roslaunch airskin_pain main.launch` to run ROS + start drivers
    - `rosrun airskin_pain main.py` to run the movement
    - `rosrun airskin_pain airskin_node.py` to run airskin events
    - `rosrun airskin_pain tester.py` to run the testing script
    - `rosrun airskin_pain eff_mass.py` to run the effective mass computation, if used

# Real setup
The code is working the same for both simulation and real setup. However, to use it in real world you need the HW
and also drivers for the HW. We can not provide the drivers here.

Drivers for robot are available at [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) 

Drivers for AIRSKIN are not publicly available (and you need the skin + special access to the HW).

# Data
All data from the experiments are stored at [OSF.io](https://osf.io/3adrm). If you want to run the same experiments as us,
please download the directory 'exps' and place it in the [data](src/airskin-pain/data) directory.
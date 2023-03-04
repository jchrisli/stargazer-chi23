This is the prototype code for the 2023 CHI paper, **Stargazer: An Interactive Camera Robot for Capturing How-To Videos Based on Subtle Instructor Cues**, by Jiannan Li, Maurício Sousa, Karthik Mahadevan, Bryan Wang, Paula Akemi Aoyaui, Nicole Yu, Angela Yang, Ravin Balakrishnan, Anthony Tang, and Tovi Grossman.

# Overview
This repository includes the software and hardware implementation of our prototype. _ros_ contains the ROS code that controls the robot (Franka Emika Panda). _sensor_ contains implementation for various sensors used, _3d_print_ includes the model of the custom camera mount for 3D printing, and _phone_ includes the custom Android camera app.

# ROS
This folder includes the ROS (Noetic) nodes for controlling the Franka Emika Panda robot. Dependencies include [MoveIt](https://github.com/ros-planning/moveit) (we make heavy use of MoveIt Servo), [Franka ROS Interface](https://github.com/justagist/franka_ros_interface), and a few others. For more information on the dependencies, see the .rosinstall file. Robot behaviors that are specific to Stargazer are mostly in packages _stargazer_ and _teleop-collab-panda-ros_.

## Change to existing packages
Note that we made several changes to existing packages to accommodate the custom requirements of our prototype system. These include:

* In _franka_panda_description_, we added the collision and appearance descriptions for our custom camera mount (_camera_mount.xarco_). This file is further integrated into _panda_arm_camera.urdf.xarco_.
* In _franka_ros_interface_, we added a custom planning scene that can be generated by running the script _create_stargazer_planning_scene.py_. Running the launch file _interface_stargazer.launch_ will execute this script along with other nodes for controlling the robot.
* In _moveit_servo_, we used `getEEFrameTransform` instead of `getCommandFrameTransform` in _pose_tracking.cpp_.

## Building and running the workspace
You will need to first create a ROS workspace and re-build all the packages. If you decide to use more recent versions of the dependencies, make sure to implement the modifications above.

## Using the Panda robot
You might want to check out the documentation for [Franka Control Interface](https://frankaemika.github.io/) and [Franka ROS Interface](https://github.com/justagist/franka_ros_interface).

# Sensors
Run `pip install` based on _requirements.txt_ to install the dependencies. _forward_kinect.py_ forward Kinect pose tracking results to the ROS node that parse pose information. _process_mp.py_ performs hand tracking and classification. You can also train your own model using the notebook _keypoint_classification_custom.ipynb_.

# 3D print
Standard STL file for the custom camera mount is included. Note its physical dimensions are for the Pixel 6 Pro phone.

# Phone app
Build and install the app to any modern Android phone. You might want to check the ip address of the phone on your network and modify the ip address in the ROS node _panda_follow_hand_ (_follow_hand.py_) accordingly. 

More details on sensor calibration to come.
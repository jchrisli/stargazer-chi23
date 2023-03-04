Relevant commands:

Launch robot sim without moveit (causes issues if launched together)
roslaunch panda_gazebo panda_world.launch start_moveit:=false

Launch move group if needed
roslaunch panda_sim_moveit sim_move_group.launch

---------------------------------

Unity-ROS communication
ROS#
roslaunch file_server ros_sharp_communication.launch

Controlling robot from Unity
rosrun teleop panda_cmds_sub.py

--------------------------------------------------

-----------------------------------------------

New Approach
roslaunch teleop unity_joint_state_sub.launch

Controlling robot from Unity
rosrun teleop panda_ik_from_joy_node.py

-------------------------------------

Movement Primitives
Server
roslaunch ros_dmp dim_sim.launch

Script to record primitive
rosrun teleop panda_create_movement_primitive.py

rosrun teleop panda_apply_movement_primitive.py

Visualize
rosrun rviz rviz
https://github.com/abakisita/ros_dmp

/learn_dmp_service_node/demonstrated_path
/generate_motion_service_node/cartesian_path

rostopic echo robot_current_fk
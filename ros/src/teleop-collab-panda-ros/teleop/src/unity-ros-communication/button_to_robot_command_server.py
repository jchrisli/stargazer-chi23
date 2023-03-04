#!/usr/bin/env python

from time import time
import rospy
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand, RobotState
from teleop_msgs.msg import SendStateAndTraj
from teleop_msgs.srv import ButtonToRobotCommand, ButtonToRobotCommandResponse, ButtonToRobotCommandRequest
from copy import deepcopy
from panda_robot import PandaArm
from franka_interface import ArmInterface
from franka_tools import JointTrajectoryActionClient
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


'''
This script allows Unity button presses to command the robot to do something; e.g. 
send it to home position OR to execute a trajectory with waypoints pre-determined.
'''

vals = []
vels = []
names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

neutral_pose = [-0.017792060227770554, -0.7601235411041661, 0.019782607023391807,
                -2.342050140544315, 0.029840531355804868,
                1.5411935298621688, 0.7534486589746342]

mvt = None
arm = None
r = None


def record_last_joint_state(msg):
    global vals , vels
    temp_vals = []
    temp_vels = []
    for n in names:
        idx = msg.name.index(n)
        temp_vals.append(msg.position[idx])
        temp_vels.append(msg.velocity[idx])

    vals = deepcopy(temp_vals)
    vels = deepcopy(temp_vels)


def respond_to_button(req):
    res = ButtonToRobotCommandResponse()
    print("Button pressed.")

    pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands', JointCommand, queue_size = 1, tcp_nodelay = True)

    # This means we 're in the home position
    if req.button_press.data == "H":
        pubmsg = JointCommand()
        pubmsg.names = names
        pubmsg.position = neutral_pose # JointCommand msg has other fields (velocities, efforts) for
        pubmsg.mode = pubmsg.POSITION_MODE
        # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)
        curr_val = deepcopy(vals)

        while all(abs(neutral_pose[i]-curr_val[i]) > 0.01 for i in range(len(curr_val))):
            pub.publish(pubmsg)
            curr_val = deepcopy(vals)
        
        res.success.data = True
        return res
        
    # This means we execute a planned trajectory
    elif req.button_press.data == "E":
        for i in range(len(req.desired_joint_trajectory.points)):
            pubmsg = JointCommand()
            pubmsg.names = names
            pubmsg.position = req.desired_joint_trajectory.points[i].positions
            pubmsg.mode = pubmsg.POSITION_MODE
            pub.publish(pubmsg)
            rospy.sleep(0.01)

        res.success.data = True
        return res
    
def main():
    rospy.init_node("button_to_robot_command_server_node")
    # To either home the robot or execute a trajectory (which we aren't really using now)
    rospy.Service("send_robot_command_service", ButtonToRobotCommand, respond_to_button)
    rospy.Subscriber('/panda_simulator/custom_franka_state_controller/joint_states', JointState, record_last_joint_state)

    print("Launched service to request robot commands from Unity.")

    rospy.spin()


if __name__ == '__main__':
    main()
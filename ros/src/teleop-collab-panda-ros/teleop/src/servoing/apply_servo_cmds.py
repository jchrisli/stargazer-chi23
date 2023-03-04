#!/usr/bin/env python3.8

import rospy
from trajectory_msgs.msg import JointTrajectory
from panda_robot import PandaArm
import time
from stargazer_msgs.msg import prop_motion


limb = None
counter = 0
window_left = time.time()

def echo(data):
    global limb
    # global counter
    # global window_left
    # limb.move_to_joint_position(data.points[0].positions)

    # limb.exec_position_cmd(data.points[0].positions)
    # print("In position tracking mode...")
    
    # if all([v == 0 for v in data.points[0].velocities]):
    #     rospy.loginfo("Immediate stop issued!")
    # else:
    limb.exec_velocity_cmd(data.points[0].velocities)
    # rospy.loginfo(f"Joint
    #  velocities {data.points[0].velocities}")

    #print("In velocity tracking mode...")

    # counter += 1
    # if counter > 300:
    #     print(f"Command freq is {300.0 / (time.time() -  window_left)} Hz")
    #     window_left = time.time()
    #     counter = 0
    # #     counter = 0
    # #     # Get EE pose
    # #     pos, rot_q = limb.ee_pose()
    # #     # print(f"Current EE pose is {pos} {rot_q}")

def prop_cmd(message):
    global limb
    data = message.motion.data
    rospy.loginfo(f"Received {data}")
    if data in ["reset"]: ## Other modes, here just for prototyping
        limb.move_to_neutral()

def main():
    
    # Set up limb
    rospy.init_node("echo_servo_node")
    global limb
    limb = PandaArm()
    limb.move_to_neutral() # always move to neutral before starting
    rospy.Subscriber('servo_server/command', JointTrajectory, echo)
    rospy.Subscriber('prop_motion', prop_motion, prop_cmd, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    main()

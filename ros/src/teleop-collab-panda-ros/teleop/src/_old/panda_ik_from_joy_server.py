#!/usr/bin/env python

import rospy
import numpy as np
from panda_robot import PandaArm
from teleop.srv import IKFromJoy, IKFromJoyResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
Return joint angles for arm movement based on provided commands through custom ros service which 
is called by a client from Unity through ROS#.
"""

panda_arm = None


def get_ik_sol(req):
    res = IKFromJoyResponse()

    # First we calculate the forward kinematics
    global panda_arm
    pos, rot = panda_arm.forward_kinematics(
        joint_angles=req.current_joint_state.position)
    
    pos = pos.flatten()

    print(pos)
    print(rot)

    if req.mode.data == True:
        # Angular
        # State in euler

        print("Angular Mode")
        rot_euler = euler_from_quaternion([rot.w, rot.x, rot.y, rot.z], axes='sxyz')
        rot_euler += np.array([req.angular.x, req.angular.y, req.angular.z])
        new_rot = quaternion_from_euler(rot_euler[0], rot_euler[1], rot_euler[2], axes='sxyz')                        
        rot = np.quaternion(new_rot[0], new_rot[1], new_rot[2], new_rot[3])

    else:
        # Linear
        pos += np.array([req.linear.x, req.linear.y, req.linear.z])

    # Calculate IK
    status, j_des = panda_arm.inverse_kinematics(pos, rot)

    res.success.data = status
    if status:
        print("Success.")
        res.updated_joint_state.position = j_des
    else:
        print("Fail.")

    #print(res.updated_joint_state)
    print("Finished processing.")
    return res

    # Then we calculate inverse kinematics with user input applied

    # Then we return the joint angles if successful


def main():
    rospy.init_node("panda_ik_from_joy")
    srv = rospy.Service("get_ik_from_joy", IKFromJoy, get_ik_sol)
    print("Advertising IK from joy service.")

    global panda_arm
    panda_arm = PandaArm()

    rospy.spin()


if __name__ == '__main__':
    main()

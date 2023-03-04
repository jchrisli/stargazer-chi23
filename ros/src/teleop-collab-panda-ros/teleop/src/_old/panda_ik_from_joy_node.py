#!/usr/bin/env python

import rospy
import numpy as np
from panda_robot import PandaArm
from teleop.msg import SendCommandUnity, ReturnCommandROS
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
Return joint angles for arm movement based on a subscriber that takes messages published
from Unity including user command and current joint angles.

sub: subscribes to user command plus current robot joint angles.
pub_fk: publishes forward kinematics for use in recording movement primitive based on current joint angles.
pub: publishes new joint angles for Unity to use to move robot.
"""

panda_arm = None
pub = pub_fk = sub = None


def get_ik_sol(msg):
    print("Message received")
    print(msg)
    res = ReturnCommandROS()

    # First we calculate the forward kinematics
    global panda_arm
    pos, rot = panda_arm.forward_kinematics(
        joint_angles=msg.current_joint_state.position)

    # Publish the forward kinematics
    # New position
    pose_msg = Pose()
    position = Point()
    position.x = pos[0]
    position.y = pos[1]
    position.z = pos[2]
    pose_msg.position = position

    # New orientation
    quat = Quaternion()
    quat.x = rot.x
    quat.y = rot.y
    quat.z = rot.z
    quat.w = rot.w
    pose_msg.orientation = quat

    # Reshape array for use
    pos = pos.flatten()

    # print(pos)
    # print(rot)

    # Angular commands - calculate new rotation
    if msg.mode.data == True:
        # Angular
        print("Angular Mode")
        rot_euler = euler_from_quaternion(
            [rot.w, rot.x, rot.y, rot.z], axes='sxyz')
        rot_euler += np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        new_rot = quaternion_from_euler(
            rot_euler[0], rot_euler[1], rot_euler[2], axes='sxyz')
        rot = np.quaternion(new_rot[0], new_rot[1], new_rot[2], new_rot[3])

    # Linear commands - calculate new position
    else:
        pos += np.array([msg.linear.x, msg.linear.y, msg.linear.z])

    # Calculate IK
    status, j_des = panda_arm.inverse_kinematics(pos, rot)

    # Return IK solution to Unity
    res.success.data = status
    if status:
        print("Success.")
        res.updated_joint_state.position = j_des
    else:
        print("Fail.")

    print(res.updated_joint_state)
    print("Finished processing.")

    global pub
    pub.publish(res)

    global pub_fk
    pub_fk.publish(pose_msg)



def main():
    rospy.init_node("panda_ik_from_joy_pub_sub")

    global pub, pub_fk, sub
    sub = rospy.Subscriber('joy_plus_joint_state',
                           SendCommandUnity, get_ik_sol, queue_size=1)
    pub = rospy.Publisher('new_joint_states', ReturnCommandROS, queue_size=1)
    pub_fk = rospy.Publisher('robot_current_fk', Pose, queue_size=1)
    print("Advertising IK from joy service.")

    global panda_arm
    panda_arm = PandaArm()
    print("LOL")
    rospy.spin()


if __name__ == '__main__':
    main()

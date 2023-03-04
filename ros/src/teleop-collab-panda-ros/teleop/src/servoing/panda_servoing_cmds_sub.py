#!/usr/bin/env python

import rospy
import numpy as np
import time
import json
from teleop_msgs.msg import MovePandaCommand
from teleop_msgs.srv import RecordStateActionPairs, RecordStateActionPairsResponse, CreateDMP
from panda_robot import PandaArm
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import random
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as t

'''
Used to send commands from Unity joystick but we now send it
to the MoveIt Servo server so that it can smooth everything 
and automatically send to the robot.
'''

# Used as callback function for any messages sent from Unity by user's joystick

class PandaTeleop():
    def __init__(self):
        self.input_thresh = 0.01

        # Input scaling
        self.scale_linear = 0.1
        self.scale_angular = 0.05

        # Setup publisher with MoveIt Servo
        self.pub = rospy.Publisher("servoing/twist_stamped_publisher", TwistStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def qv_mult(self, q1, v1):
        q1 = [q1.x, q1.y, q1.z, q1.w]
        q2 = [v1.x, v1.y, v1.z, 0.0]
        return t.quaternion_multiply(
            t.quaternion_multiply(q1, q2),
            t.quaternion_conjugate(q1))[:3]

    def transform_twist(self, twist_rot, twist_vel):
        transform = self.tf_buffer.lookup_transform("world", "panda_hand", rospy.Time(),
                                                                         rospy.Duration(1.0))
        map_to_ee_trans = transform.transform.translation
        map_to_ee_rot = transform.transform.rotation

        q1 = map_to_ee_rot
        out_vel = self.qv_mult(q1, twist_vel)
        out_rot = twist_rot

        return out_vel, out_rot

    def send_joy_to_servo(self, msg):
        new_twist = TwistStamped()
        new_twist.header.stamp = rospy.Time.now()
        new_twist.header.frame_id = 'panda_link0'

        # If input doesn't meet threshold don't send commands
        #if (abs(msg.x_axis.data) < self.input_thresh and abs(msg.y_axis.data) < self.input_thresh and
            #abs(msg.z_axis.data) < self.input_thresh):
            #return
        
        # Provide linear or angular commands (depending on what user sends)
        if msg.angular_control.data == False:
            new_twist.twist.linear.x = msg.x_axis.data
            new_twist.twist.linear.y = msg.y_axis.data
            new_twist.twist.linear.z = msg.z_axis.data

            new_twist.twist.angular.x = 0
            new_twist.twist.angular.y = 0
            new_twist.twist.angular.z = 0

        else:
            new_twist.twist.angular.x = msg.y_axis.data
            new_twist.twist.angular.y = msg.z_axis.data
            new_twist.twist.angular.z = msg.x_axis.data

            new_twist.twist.linear.x = 0
            new_twist.twist.linear.y = 0
            new_twist.twist.linear.z = 0


        transform = self.tf_buffer.lookup_transform('world', 'panda_hand', rospy.Time(0), rospy.Duration(1.0))
        old_linear_vec = Vector3Stamped()
        old_linear_vec.vector.x = new_twist.twist.linear.x
        old_linear_vec.vector.y = new_twist.twist.linear.y
        old_linear_vec.vector.z = new_twist.twist.linear.z
        new_vel = tf2_geometry_msgs.do_transform_vector3(old_linear_vec, transform)
        #out_vel, out_rot = self.transform_twist(new_twist.twist.linear, new_twist.twist.angular)
        new_twist.twist.linear.x = new_vel.vector.x
        new_twist.twist.linear.y = new_vel.vector.y
        new_twist.twist.linear.z = new_vel.vector.z
        #new_twist.twist.angular.x = out_rot.x
        #new_twist.twist.angular.y = out_rot.y
        #new_twist.twist.angular.z = out_rot.z

        self.pub.publish(new_twist)


def main():
    rospy.init_node("panda_servoing_from_unity_joy_node")
    print("Listening for Unity joy input.")

    panda_teleop = PandaTeleop()

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.Subscriber("/publish_panda_cmds", MovePandaCommand,
                     panda_teleop.send_joy_to_servo, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()

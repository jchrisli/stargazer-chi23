#!/usr/bin/env python

"""
Script to allow subscription to Unity for a trajectory which we can then learn from and generate a DMP
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from ros_dmp.srv import *

sub = None
sub_record = None
record = False


def record_primitive(msg):
    poses_array = PoseArray()
    poses_array.poses = msg.poses
    if len(poses_array.poses) > 0:
        print("Saving primitive.")
        # Save the primitive
        try:
            # Create the request
            req = LearnDMPRequest()
            req.header.frame_id = 'base'
            req.output_weight_file_name = 'unity_primitive.yaml'
            req.dmp_name = 'square_wave'
            req.header.stamp = rospy.Time.now()
            req.n_bfs = 500
            req.n_dmps = 6

            for i in range(len(poses_array.poses)):
                req.poses.append(poses_array.poses[i])

            service_client = rospy.ServiceProxy(
                '/learn_dynamic_motion_primitive_service', LearnDMP)
            rospy.loginfo(service_client(req))
        except:
            rospy.loginfo("Service call to save primitive failed.")

        # Reset poses array
        poses_array = []


def main():
    global sub, sub_record
    rospy.init_node("record_dmp_node")
    sub_record = rospy.Subscriber(
        'record_primitive', PoseArray, record_primitive)
    print("Can now record dynamic movement primitives.")
    rospy.spin()


if __name__ == '__main__':
    main()

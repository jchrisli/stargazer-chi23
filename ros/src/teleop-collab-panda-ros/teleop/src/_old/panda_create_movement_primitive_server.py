#!/usr/bin/env python

"""
Script to allow Unity to call a service to record a movement primitive (DMP)
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from ros_dmp.srv import *
from teleop_msgs.srv import CreateDMP, CreateDMPResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory
from rospy_message_converter import message_converter
import json


def record_primitive(dmp_creation_req):
    dmp_creation_res = CreateDMPResponse()
    if len(dmp_creation_req.waypoint_poses) > 0:
        print("Saving primitive.")
        print("Num waypoints record: ", len(dmp_creation_req.waypoint_poses))

        ### Also save demo properly for ProMP future
        promp_path = Path()
        for i in range(len(dmp_creation_req.waypoint_poses)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base'
            pose_stamped.pose = dmp_creation_req.waypoint_poses[i]
            promp_path.poses.append(pose_stamped)

        # Slice trajectory to include only the arm joints
        promp_traj = JointTrajectory()
        promp_traj.joint_names = dmp_creation_req.robot_traj.joint_trajectory.joint_names[2:]
        promp_traj.points = dmp_creation_req.robot_traj.joint_trajectory.points
        for i in range(len(promp_traj.points)):
            promp_traj.points[i].positions = promp_traj.points[i].positions[2:]
            promp_traj.points[i].velocities = promp_traj.points[i].velocities[2:]
            promp_traj.points[i].accelerations = promp_traj.points[i].accelerations[2:]
            promp_traj.points[i].effort = promp_traj.points[i].effort[2:]

        # Save the DMP primitive
        try:
            # Create the request
            req = LearnDMPRequest()
            req.header.frame_id = 'base'
            req.output_weight_file_name = dmp_creation_req.primitive_name.data + '.yaml'
            req.dmp_name = 'square_wave'
            req.header.stamp = rospy.Time.now()
            req.n_bfs = 500
            req.n_dmps = 6
            req.poses = dmp_creation_req.waypoint_poses

            service_client = rospy.ServiceProxy(
                '/learn_dynamic_motion_primitive_service', LearnDMP)
            rospy.loginfo(service_client(req))
            
            dmp_creation_res.success.data = True
            return dmp_creation_res
            
        except:
            rospy.loginfo("Service call to save primitive failed.")


def main():
    rospy.init_node("record_dmp_node")
    srv = rospy.Service("poses_to_new_MP", CreateDMP, record_primitive)
    print("Advertising DMP creation service.")
    rospy.spin()


if __name__ == '__main__':
    main()

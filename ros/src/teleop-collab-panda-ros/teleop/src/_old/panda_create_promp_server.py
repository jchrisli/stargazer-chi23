#!/usr/bin/env python

"""
Script to allow Unity to call a service to record a movement primitive (DMP)
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ros_dmp.srv import *
from teleop.srv import CreateDMP, CreateDMPRequest, CreateDMPResponse
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory
from promp.ros import TaskProMP
from teleop.srv import PosesToDMP, PosesToDMPResponse
from rospy_message_converter import message_converter
import json


def record_primitive(dmp_creation_req):
    dmp_creation_res = CreateDMPResponse()
    # print(dmp_creation_req)
    if len(dmp_creation_req.waypoint_poses) > 0:
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

        # Save the ProMP
        if len(dmp_creation_req.waypoint_poses) > 0 and len(promp_traj.points) > 0:

            promp_demo = message_converter.convert_ros_message_to_dictionary(dmp_creation_req)
            promp_filename = dmp_creation_req.primitive_name.data

            with open("/home/karthikm/catkin_ws/src/teleop-collab-panda-ros/teleop/src/saved_promps/" + promp_filename + ".json", 'w') as f:
                json.dump(promp_demo, f)
                print("Saved demo to drive.")

    dmp_creation_res.success.data = True
    return dmp_creation_res


def main():
    rospy.init_node("record_promp_node")
    srv = rospy.Service("poses_to_new_MP", CreateDMP, record_primitive)
    '''srv = rospy.Service("poses_to_DMP_service",
                        PosesToDMP, generate_trajectory)'''
    print("Advertising DMP creation service.")
    rospy.spin()


if __name__ == '__main__':
    main()
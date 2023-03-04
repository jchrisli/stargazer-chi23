#!/usr/bin/env python

import rospy
from franka_moveit import PandaMoveGroupInterface
from geometry_msgs.msg import Pose
from panda_motion_plan_execute_server import PlanAndExecuteMotion

_mg = None

def seg_mg():
    global _mg
    _mg = PandaMoveGroupInterface()


def get_mg():
    return _mg


def get_motion_plan(poses):
    plan, fraction = _mg.plan_cartesian_path(poses)
    print("Got motion plan")
    return plan, fraction

rospy.init_node("move_group_interface_node")
seg_mg()
rospy.spin()

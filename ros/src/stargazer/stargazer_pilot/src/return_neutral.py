#!/usr/bin/env python3.8
## Go back to the neutral pose

import rospy
from panda_robot import PandaArm
from time import sleep
import numpy as np
import quaternion
import tf2_ros



class PosControDemo(object):

    def __init__(self) -> None:
        rospy.init_node("panda_demo") # initialise ros node
        self._arm = PandaArm()
        # self._poses = [([ 0.32834225, -0.18669994,  0.69329911], np.quaternion(0.128400148400615, -0.878867090918847, 0.347607488990834, -0.300458103548616)),
        # ([0.39227784, -0.03403197, 0.73494872], np.quaternion(0.137412357300329, -0.871360686446298, 0.344715347976336, -0.32096686284811)),
        # ([0.43203484, 0.17899153, 0.67301246], np.quaternion(0.0978350174403839, -0.878388505515991, 0.331933456038258, -0.329669718813121)),
        # ([0.39227784, -0.03403197, 0.73494872], np.quaternion(0.137412357300329, -0.871360686446298, 0.344715347976336, -0.32096686284811)),
        # ([0.32834225, -0.18669994,  0.69329911], np.quaternion(0.128400148400615, -0.878867090918847, 0.347607488990834, -0.300458103548616))]
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def show_off(self) -> None:
        #count = 0
        print(f"Initial joint position is {self._arm.angles()}")
        self._arm.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client


        # pos,ori = self._arm.ee_pose() # get current end-effector pose (3d position and orientation quaternion of end-effector frame in base frame)
        # print(f"Returned to neutral pose {pos} {ori}")

        # r.get_gripper().home_joints() # homes gripper joints
        # r.get_gripper().open() # open gripper

        #r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01]) # move robot to the specified pose

def main():
    try:
        demo = PosControDemo()
        demo.show_off()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()


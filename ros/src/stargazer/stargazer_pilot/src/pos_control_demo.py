#!/usr/bin/env python3.8
## Try with Panda Robot

import rospy
from panda_robot import PandaArm
from time import sleep
import numpy as np
import quaternion


class PosControDemo(object):

    def __init__(self) -> None:
        rospy.init_node("panda_demo") # initialise ros node
        self._arm = PandaArm()
        self._poses = [([ 0.32834225, -0.18669994,  0.69329911], np.quaternion(0.128400148400615, -0.878867090918847, 0.347607488990834, -0.300458103548616)),
        ([0.39227784, -0.03403197, 0.73494872], np.quaternion(0.137412357300329, -0.871360686446298, 0.344715347976336, -0.32096686284811)),
        ([0.43203484, 0.17899153, 0.67301246], np.quaternion(0.0978350174403839, -0.878388505515991, 0.331933456038258, -0.329669718813121)),
        ([0.39227784, -0.03403197, 0.73494872], np.quaternion(0.137412357300329, -0.871360686446298, 0.344715347976336, -0.32096686284811)),
        ([0.32834225, -0.18669994,  0.69329911], np.quaternion(0.128400148400615, -0.878867090918847, 0.347607488990834, -0.300458103548616))]

    def show_off(self) -> None:
        count = 0
        # self._arm.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client
        cycle = len(self._poses)

        # pos,ori = r.ee_pose() # get current end-effector pose (3d position and orientation quaternion of end-effector frame in base frame)

        # r.get_gripper().home_joints() # homes gripper joints
        # r.get_gripper().open() # open gripper

        #r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01]) # move robot to the specified pose

        while not rospy.is_shutdown():
            #self._arm.move_to_cartesian_pose(self._poses[count % cycle][0], self._poses[count % cycle][1]) # move the robot end-effector to pose specified by 'pos','ori'
            pos = self._poses[count % cycle][0]
            rot = self._poses[count % cycle][1]
             
            success, joint_command = self._arm.inverse_kinematics(pos, rot)
            print(joint_command)
            if success:
               self._arm.move_to_joint_position(joint_command)

             #self._arm

             # sleep(2.0)
            sleep(0.3)
            count += 1

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


#!/usr/bin/env python3.8

'''
Control arm movement
'''

import rospy
# from time import sleep
import numpy as np
import numpy.linalg as linalg
import quaternion
import tf2_ros
import geometry_msgs.msg
from panda_robot import PandaArm


class ArmGoGo(object):

    def __init__(self) -> None:
        rospy.init_node("panda_arm_gogo") # initialise ros node
        self._arm = PandaArm()
        # self._arm.set_arm_speed(0.1)
    
        self._goto_sub = rospy.Subscriber('robot_action/goto', geometry_msgs.msg.Transform, self.__goto_transform)
        rospy.spin()

    def __goto_transform(self, trans: geometry_msgs.msg.Transform) -> None:
        # TODO: Do something to verify it's safe?
        # TODO: for now, do not change in rotation
        # curr_p, curr_r = self._arm.ee_pose()
        print(f'Going to {trans}')

        # Do joint space planning instead of Cartesian space
        success, joint_command = self._arm.inverse_kinematics([trans.translation.x, trans.translation.y, trans.translation.z],
                                np.quaternion(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z))

        if success:
            print(joint_command)
            self._arm.move_to_joint_position(joint_command)
        else:
            print('IK failed, sad.')

        # self._arm.move_to_cartesian_pose([trans.translation.x, trans.translation.y, trans.translation.z],
        #                                np.quaternion(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z)) # move the robot end-effector to pose specified by 'pos','ori'

def main():
    try:
        arm_go = ArmGoGo()
        #follow.listen(hand_name='tracked_hand')
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
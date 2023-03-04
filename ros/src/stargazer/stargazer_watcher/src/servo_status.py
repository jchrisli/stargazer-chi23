#!/usr/bin/env python3.8

'''
Track MoveIt servo status and reset the robot if it's stuck
'''
from std_msgs.msg import Int8
from stargazer_msgs.msg import prop_motion

class ServoStatus(object):
    def __init__(self) -> None:
        rospy.init_node("servo_status")
        self._status_sub = rospy.Subscriber('status', Int8, self.__process_status)
        self._reset_pub = rospy.Publisher('prop_motion', prop_motion, queue_size=10)

    def __process_status(self, status: int) -> None:
        ## See https://github.com/ros-planning/moveit/blob/master/moveit_ros/moveit_servo/include/moveit_servo/status_codes.h
        ## If the robot halts for singularity or collision, try reset
        if status == 2 or status == 4:
            msg = prop_motion()
            msg.motion.data = 'reset'
            self._reset_pub.publish(msg)
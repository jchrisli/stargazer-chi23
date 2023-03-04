#!/usr/bin/env python3.8

'''
Decouple sensing and actuation for prototyping
'''

from ast import Str
import rospy
from skeleton_tracker import KinectUdpListener
## import std_msgs.msg
from stargazer_msgs.msg import prop_motion
import getch

class PropTracker(object):
    def __init__(self) -> None:
        self._port = 12348
        self._udp_listener = KinectUdpListener(self._port, timeout=3.0) 
        self._node = rospy.init_node('prop_tracker')
        self._prop_pub = rospy.Publisher('prop_motion', prop_motion, queue_size=10)

    @staticmethod
    def make_prop_motion_msg(action: Str) -> prop_motion:
        msg = prop_motion()
        msg.motion.data = action
        return msg

    def read_data_and_publish(self) -> None:
        key = getch.getch()
        # print(f"Got key {key}")
        try:
            k = int(key)
        except ValueError:
            rospy.logerr(f"Invalid keyboard input {key}")
            return
        # k = 10
        if k == 0: # numpad 0
            m = PropTracker.make_prop_motion_msg("reset")
            self._prop_pub.publish(m)
            rospy.loginfo("Reset request sent")
        elif k == 1: # numpad 1
            m = PropTracker.make_prop_motion_msg("pan")
            self._prop_pub.publish(m)
            rospy.loginfo("Pan request sent")
        elif k == 2:
            m = PropTracker.make_prop_motion_msg("top")
            self._prop_pub.publish(m)
            rospy.loginfo("Top request sent")
        elif k == 3:
            m = PropTracker.make_prop_motion_msg("level")
            self._prop_pub.publish(m)
            rospy.loginfo("Level request sent")
        elif k == 4:
            m = PropTracker.make_prop_motion_msg("close")
            self._prop_pub.publish(m)
            rospy.loginfo("Close request sent")
        elif k == 5:
            m = PropTracker.make_prop_motion_msg("medium")
            self._prop_pub.publish(m)
            rospy.loginfo("Medium request sent")
        elif k == 7:
            m = PropTracker.make_prop_motion_msg("orbit")
            self._prop_pub.publish(m)
            rospy.loginfo("Orbit request sent")
        elif k == 8:
            m = PropTracker.make_prop_motion_msg("left")
            self._prop_pub.publish(m)
            rospy.loginfo("Left request sent")
        elif k == 9:
            m = PropTracker.make_prop_motion_msg("right")
            self._prop_pub.publish(m)
            rospy.loginfo("Right request sent")

if __name__ == '__main__':
    pt = PropTracker()
    while not rospy.is_shutdown():
        try:
            pt.read_data_and_publish()
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            print("Exit because of keyboard interruption")
            break

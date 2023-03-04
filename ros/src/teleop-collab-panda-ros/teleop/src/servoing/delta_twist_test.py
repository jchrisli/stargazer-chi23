#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import TwistStamped


class DeltaTwistTest(object):
    def __init__(self) -> None:
        
        self._node = rospy.init_node("delta_twist_test") # initialise ros node
        self._rate = rospy.Rate(100.0)
        self._delta_twist_pub = rospy.Publisher("servo_server/delta_twist_cmds", TwistStamped, queue_size=1)

    def go(self) -> None:
        while not rospy.is_shutdown():
            new_twist = TwistStamped()
            new_twist.header.stamp = rospy.Time.now()
            new_twist.header.frame_id = 'panda_link0'
            # new_twist.twist.linear.x = 0.14535859817895422
            # new_twist.twist.linear.y = 0.007969376617643697
            # new_twist.twist.linear.z = 0.3423814206975754

            # new_twist.twist.angular.x = 0.6135269365266334
            # new_twist.twist.angular.y = -0.449319158157156
            # new_twist.twist.angular.z = -0.1333256120456457

            new_twist.twist.linear.x = 1
            new_twist.twist.linear.y = 1
            new_twist.twist.linear.z = 0

            new_twist.twist.angular.x = 0
            new_twist.twist.angular.y = 0
            new_twist.twist.angular.z = 1

            self._delta_twist_pub.publish(new_twist)
            self._rate.sleep()
    

if __name__ == '__main__':
    test = DeltaTwistTest()
    test.go()
    

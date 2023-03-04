#!/usr/bin/env python3.8

import queue
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('visualize')

p = rospy.Publisher('/visualize_pose', PoseStamped, queue_size=1)
pose = PoseStamped()
pose.header.frame_id = 'world'
pose.pose.position.x = 0.687
pose.pose.position.y = 0.065
pose.pose.position.z = -0.012
pose.pose.orientation.x = 0.04
pose.pose.orientation.y = 0.011
pose.pose.orientation.y = 0.111
pose.pose.orientation.z = 0.99
pose.pose.orientation.w  = -0.017


# try:
while not rospy.is_shutdown():

    p.publish(pose)
    rospy.sleep(0.5)
# except rospy.ROSInterruptException, :
#     return
# except KeyboardInterrupt:
#     return


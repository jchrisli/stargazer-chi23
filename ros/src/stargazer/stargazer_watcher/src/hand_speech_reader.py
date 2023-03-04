#!/usr/bin/env python3.8

import rospy
from skeleton_tracker import KinectUdpListener
import json

#import tf2_geometry_msgs
#import geometry_msgs.msg
import std_msgs.msg
import tf2_ros

class HandSpeechReader(object):
    '''
    Read MediaPipe detection results of hand gestures and publish ROS messages
    '''
    def __init__(self) -> None:
        self._port = 12347
        self._udp_listener = KinectUdpListener(self._port, timeout=3.0) 
        self._node = rospy.init_node('hand_speech_reader')
        #self._hand_normal_publisher = rospy.Publisher('hand_normal', geometry_msgs.msg.Vector3Stamped, queue_size=1)
        self._hand_speech_pub = rospy.Publisher('hand_speech_signal', std_msgs.msg.String, queue_size=1)
        self._tfBuffer = tf2_ros.Buffer() ## Looking up transform from Kinect to base
        self._listener = tf2_ros.TransformListener(self._tfBuffer)


    def read_data_and_publish(self) -> None:
        '''
        Reading hand pointing and face direction messages and do some filtering
        '''
        try:
            data = self._udp_listener.listen()
        except ConnectionError:
            # print(f"Not receiving any MediaPipe data through UDP. Try again?")
            return
        shot_type = json.loads(data.decode('utf-8'))["shotType"]
        hand_speech_msg = std_msgs.msg.String()
        hand_speech_msg.data = shot_type
        self._hand_speech_pub.publish(hand_speech_msg)

if __name__ == '__main__':
    hsr = HandSpeechReader()
    while not rospy.is_shutdown():
        try:
            hsr.read_data_and_publish()
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            print("Exit because of keyboard interruption")
            break

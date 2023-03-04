#!/usr/bin/env python3.8

import socket
from typing import Callable, Dict

import numpy as np
import rospy
import re
import tf2_ros
import geometry_msgs.msg



class KinectUdpListener(object):

    def __init__(self, port: int, timeout=5.0) -> None:
        self._port = port
        self._receive_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        #self._receive_socket.bind(('0.0.0.0', int(self._port)))
        #self._receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._receive_socket.bind(('', int(self._port))) # bind to broadcast address
        if timeout > 0:
            self._receive_socket.settimeout(timeout)

    def listen(self) -> bytes:
        try:
            data, _addr = self._receive_socket.recvfrom(20000)
            return data
        except socket.timeout:
            # print('Timeout when listening to Kinect UDP data, is there data?')
            raise ConnectionError

class Vector3Filter(object):
    '''
    Simple low-pass filter for 3-vector using a fixed coefficient
    '''
    def __init__(self, co: float = 0.5) -> None:
        '''
        co: how much we trust new data
        '''
        self._val = np.array([0, 0, 0], dtype=np.double)
        self._initialized = False
        self._co = co
        
    def update(self, new_val: np.ndarray) -> np.ndarray:
        if not self._initialized:
            self._val = new_val
        else:
            self._val = self._val * (1 - self._co) + new_val * self._co
        return self._val

    def get_val(self) -> np.ndarray:
        return self._val 


class SkeletonTracker(object):
    '''
    Read broadcasted data from CreepyTracker over Ethernet and locate hands and head
    '''
    def __init__(self) -> None:
        #self._UDP_PORT = 5102
        self._UDP_PORT = 12345
        self._udp_listener = KinectUdpListener(self._UDP_PORT)
        self._kinect_frame = 'kinect'
        # self._base_frame_name = 'panda_link0'
        self._node = rospy.init_node("stargazer_skeleton_tracker")
        self._br = tf2_ros.TransformBroadcaster()
        # self._parent_frame = parent_frame
        self._left_hand_frame = 'person_left_hand'
        self._right_hand_frame = 'person_right_hand'
        self._head_frame = 'person_head'
        self._left_shoulder_frame = 'person_left_shoulder'
        self._right_shoulder_frame = 'person_right_shoulder'
        self._left_wrist_frame = 'person_left_wrist'
        self._right_wrist_frame = 'person_right_wrist'
        self._spine_base_frame = 'person_spine_base'
        self._spine_mid_frame = 'person_spine_mid'

        self._left_hand_filter = Vector3Filter(0.5)
        self._right_hand_filter = Vector3Filter(0.5)
        
    def __construct_tf_msg(self, trans_v: np.ndarray, target_frame: str):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._kinect_frame # parent id
        t.child_frame_id = target_frame
        # print(f"shape of trans_v is {trans_v.shape}")
        t.transform.translation.x = trans_v[0]
        t.transform.translation.y = trans_v[1]
        t.transform.translation.z = trans_v[2]
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        # print(f"Publishing kinect tf {t}")

        return t

    def read_data_and_publish(self) -> None:
        '''
        Listen to UDP and publish TF transforms for hands and head
        '''
        try:
            data = self._udp_listener.listen()
            skeleton_info = self.__parse_kinect_data(data)
            ## Filter hand positions 
            if 'l' in skeleton_info:
                skeleton_info['l'] = self._left_hand_filter.update(skeleton_info['l'])
            if 'r' in skeleton_info:
                skeleton_info['r'] = self._right_hand_filter.update(skeleton_info['r'])

            for key_str, frame_name in [('l', self._left_hand_frame), ('r', self._right_hand_frame), ('h', self._head_frame),
                                         ('ls', self._left_shoulder_frame), ('rs', self._right_shoulder_frame),
                                         ('lw', self._left_wrist_frame), ('rw', self._right_wrist_frame), 
                                         ('sm', self._spine_mid_frame), ('sb', self._spine_base_frame)]:
                if key_str in skeleton_info:
                    self._br.sendTransform(self.__construct_tf_msg(skeleton_info[key_str], frame_name))
        except ConnectionError:
            print(f"Not receiving any skeleton data through UDP. Try again?")

    def __decode_creepy_tracker_message(self, message: str)-> Dict[str, np.ndarray]:
        left_hand_name = 'leftHandTip'
        right_hand_name = 'rightHandTip'
        head_name = 'head'
        left_shoulder_name = 'leftShoulder'
        right_shoulder_name = 'rightShoulder'
        left_wrist_name = 'leftWrist'
        right_wrist_name = 'rightWrist'
        spine_base_name = 'spineBase'
        spine_mid_name = 'spineMid'

        ret = {}
        if len(message) > 0: # so at least one skeleton found
            # print(message)
            left_hand_match = re.search(f'(?<={left_hand_name}=)[0-9-.:]+', message)
            right_hand_match = re.search(f'(?<={right_hand_name}=)[0-9-.:]+', message)
            head_match = re.search(f'(?<={head_name}=)[0-9-.:]+', message)
            left_shoulder_match = re.search(f'(?<={left_shoulder_name}=)[0-9-.:]+', message)
            right_shoulder_match = re.search(f'(?<={right_shoulder_name}=)[0-9-.:]+', message)
            left_wrist_match = re.search(f'(?<={left_wrist_name}=)[0-9-.:]+', message)
            right_wrist_match = re.search(f'(?<={right_wrist_name}=)[0-9-.:]+', message)
            spine_base_match = re.search(f'(?<={spine_base_name}=)[0-9-.:]+', message)
            spine_mid_match = re.search(f'(?<={spine_mid_name}=)[0-9-.:]+', message)

            for matched, key in [(left_hand_match, 'l'), (right_hand_match, 'r'), (head_match, 'h'),
                                 (left_shoulder_match, 'ls'), (right_shoulder_match, 'rs'), (left_wrist_match, 'lw'),
                                 (right_wrist_match, 'rw'), (spine_base_match, 'sb'), (spine_mid_match, 'sm')]:
                if matched:
                    numbers = [float(n) for n in matched.group(0).split(':')]
                    ret[key] = np.array(numbers)
        return ret


    def __parse_kinect_data(self, data: bytes) -> Dict[str, np.ndarray]:
        '''
            parse the broadcasted data and return hand (left and then right) and head positions
        '''
        dataStr = data.decode("utf-8")
        # Flip the x axis of the Kinect data to make it right-handed
        # print(f"Received {data}")


        decoded = self.__decode_creepy_tracker_message(dataStr)
        for k in decoded:
            decoded[k][0] = -decoded[k][0]
        # print(decoded)

        return decoded

        
if __name__ == '__main__':
    skeleton_tracker = SkeletonTracker()
    try:
        while not rospy.is_shutdown():
            skeleton_tracker.read_data_and_publish()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Ended because of keyboard interruption.")

#!/usr/bin/env python3.8

import rospy
from skeleton_tracker import KinectUdpListener
from stargazer_msgs.msg import hand_gesture, facing_EE
import json
from stargazer_msgs.msg import prop_motion

# import tf2_geometry_msgs
# import geometry_msgs.msg
import tf2_ros
import numpy as np

class BinaryFilter(object):
    '''
    Filtering binary state streams by converting them to continuous values 
    '''
    def __init__(self, init_val: bool, filter_co: float = 0.25, filter_T_co: float = -1, filter_F_co = -1) -> None:
        #self._F2T_threshold = 0.7
        #self._T2F_threshold = 0.3
        self._val = init_val ## Might be useful if we want to use two different thresholds for state transitions
        self._cont_val = 1.0 if init_val else 0.0
        self._biased = filter_T_co > 0 and filter_F_co > 0
        if self._biased:
            self._filter_T_co = filter_T_co # how much we trust new true value
            self._filter_F_co = filter_F_co # how much we trust new false value
        else:
            self._filter_co = filter_co # how much we trust new value

    def update(self, new_val: bool) -> bool:
        filter_co = (self._filter_T_co if new_val else self._filter_F_co) if self._biased else self._filter_co
        self._cont_val = self._cont_val * (1 - filter_co) + (1.0 if new_val else 0.0) * filter_co
        return self._cont_val >= 0.5

    def get_val(self) -> bool:
        return self._cont_val >= 0.5

    def hard_update(self, new_val: bool) -> bool:
        self._cont_val = 1.0 if new_val else 0.0
        return self._cont_val >= 0.5

class MediaPipeReader(object):
    '''
    Read MediaPipe detection results of hand gestures and publish ROS messages
    '''
    def __init__(self) -> None:
        self._port = 12346
        self._udp_listener = KinectUdpListener(self._port)
        self._node = rospy.init_node('mediapipe_reader')
        self._pipe_pointing_publisher = rospy.Publisher('mediapipe_pointing', hand_gesture, queue_size=1)
        self._pipe_facing_publisher = rospy.Publisher('mediapipe_facing', facing_EE, queue_size=1)
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._prop_sub = rospy.Subscriber('prop_motion', prop_motion, self.__prop_cb, queue_size=10)

        # ------- pointing gestures ------------
        #self._prev_pointing_data = '' ## can be 'Left', 'Right', ''
        #self._pointing_count = 0
        self._left_pointing_filter = BinaryFilter(init_val=False, filter_T_co=0.1, filter_F_co=0.07) 
        self._right_pointing_filter = BinaryFilter(init_val=False, filter_T_co=0.1, filter_F_co=0.07)
        # self._pointing_detected = '' ## can be 'Left', 'Right', ''
        self._prop_pointing_left = False
        self._prop_pointing_right = False

        # -------- facing directions -----------
        # self._looking_into_eyes = False
        #self._looking_into_eyes_count = 0
        #self._prev_looking_into_eyes = False
        self._facing_angle_thres = np.cos(np.pi / 8.0)
        self._prev_looking_into_eyes_vec = None # previous filtered face orientation, normalized
        self._looking_into_eyes_vec_filter_co = 0.99 # how much do we trust new measurement?
        self._looking_into_eyes_filter = BinaryFilter(init_val=False, filter_T_co=0.02, filter_F_co=0.05)
        self._FOV = np.pi / 180.0 * 60.0
        self._ASPECT_RATIO = 16.0 / 9.0
        self._looking_into_eyes_changed_since = -1
        self._looking_into_eyes_state_at_least = 5.0
        # self._pointing_in_center_since_ts = -1
        

    @staticmethod
    def obj_in_fov_center(p_cam_frame: np.ndarray, h_fov: float, aspect_ratio: float, portion: float) -> bool:
        '''
        Check if an object is in the center (according to the `portion` param provided) of the camera frame
        '''
        px, py, pz = p_cam_frame[0], p_cam_frame[1], p_cam_frame[2]
        return np.abs(px / pz) / np.tan(h_fov / 2.0) < portion and np.abs(py / pz) / np.tan(h_fov / 2.0) / aspect_ratio < portion

    def __prop_cb(self, prop_motion_msg: prop_motion) -> None:
        data = prop_motion_msg.motion.data
        if data == 'left':
            self._prop_pointing_left = not self._prop_pointing_left
            rospy.loginfo(f"Setting left pointing to be {self._prop_pointing_left} from prop input")
        elif data == 'right':
            self._prop_pointing_right = not self._prop_pointing_right
            rospy.loginfo(f"Setting left pointing to be {self._prop_pointing_right} from prop input")

    def read_data_and_publish(self) -> None:
        '''
        Reading hand pointing and face direction messages and do some filtering
        '''
        try:
            data = self._udp_listener.listen()
            mediapipe_info = json.loads(data.decode('utf-8'))

        except ConnectionError:
            print(f"Not receiving any MediaPipe data through UDP. Try again?")
            return

        #-------- hand pointing -----------
        was_left_pointing = self._left_pointing_filter.get_val()
        was_right_pointing = self._right_pointing_filter.get_val()
        prev_pointing = 'Both' if (was_left_pointing and was_right_pointing) else ('Left' if was_left_pointing else ('Right' if was_right_pointing else ''))
        # curr_left_pointing = self._left_pointing_filter.update('Left' in mediapipe_info['handedness'])
        # curr_right_pointing = self._right_pointing_filter.update('Right' in mediapipe_info['handedness'])
        curr_left_pointing = self._left_pointing_filter.update('Left' in mediapipe_info['handedness'] or self._prop_pointing_left)
        curr_right_pointing = self._right_pointing_filter.update('Right' in mediapipe_info['handedness'] or self._prop_pointing_right)
        curr_pointing = 'Both' if (curr_left_pointing and curr_right_pointing) else ('Left' if curr_left_pointing else ('Right' if curr_right_pointing else ''))

        # Only publish when the pointing state changes
        if curr_pointing != prev_pointing:
            mediapipe_pointing_msg = hand_gesture()
            mediapipe_pointing_msg.handedness.data = curr_pointing
            self._pipe_pointing_publisher.publish(mediapipe_pointing_msg)

        # ------------- face orientation -------------
        if rospy.get_time() - self._looking_into_eyes_changed_since > self._looking_into_eyes_state_at_least: ## Avoid too frequent state changes
        ## ------------ hard set to true if condition met ---------
            prev_look_into_eyes = self._looking_into_eyes_filter.get_val()
            if not prev_look_into_eyes:
                try:
                    head_trans = self._tfBuffer.lookup_transform('panda_link0', 'person_head', rospy.Time())
                    left_hand_trans = self._tfBuffer.lookup_transform('panda_link0', 'person_left_hand', rospy.Time())
                    right_hand_trans = self._tfBuffer.lookup_transform('panda_link0', 'person_right_hand', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print(f"Cannot look up transform")
                    ## Do not change anything upon exceptions, hopefully future data can help make decisions?
                    return 
                if left_hand_trans.transform.translation.z > head_trans.transform.translation.z or right_hand_trans.transform.translation.z > head_trans.transform.translation.z:
                    self._looking_into_eyes_filter.update(True)
            else: 

                # if mediapipe_info['face_found'] == False:
                #     self._looking_into_eyes_filter.update(False)
                #     self._prev_looking_into_eyes_vec = None
                # else:
                #     face_orientation_in_ext_cam = geometry_msgs.msg.Vector3Stamped()
                #     face_orientation_in_ext_cam.header.frame_id = "c922_camera"
                #     face_orientation_in_ext_cam.vector.x = mediapipe_info['face_orientation'][0]
                #     face_orientation_in_ext_cam.vector.y = mediapipe_info['face_orientation'][1]
                #     face_orientation_in_ext_cam.vector.z = mediapipe_info['face_orientation'][2]

                #     EE_origin = geometry_msgs.msg.PointStamped()
                #     EE_origin.header.frame_id = 'pixel6pro' ## Maybe use a different link?
                #     EE_origin.point.x, EE_origin.point.y, EE_origin.point.z = 0.0, 0.0, 0.0
                #     try:
                #         ext_cam_to_head_trans = self._tfBuffer.lookup_transform("person_head", "c922_camera", rospy.Time())
                #         EE_to_head_trans = self._tfBuffer.lookup_transform("person_head", "pixel6pro", rospy.Time())
                #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #         print(f"Cannot look up transform for c922_camera or pixel6pro to person_head")
                #         ## Do not change anything upon exceptions, hopefully future data can help make decisions?
                #         return 
                #     face_orientation_in_head = tf2_geometry_msgs.do_transform_vector3(face_orientation_in_ext_cam, ext_cam_to_head_trans)
                #     EE_direction_in_head = tf2_geometry_msgs.do_transform_point(EE_origin, EE_to_head_trans)
                #     EE_direction_in_head_np = np.array([EE_direction_in_head.point.x, EE_direction_in_head.point.y, EE_direction_in_head.point.z])
                #     EE_direction_in_head_np = EE_direction_in_head_np / np.linalg.norm(EE_direction_in_head_np)
                #     face_orientation_in_head_np = np.array([face_orientation_in_head.vector.x, face_orientation_in_head.vector.y, face_orientation_in_head.vector.z])
                #     if self._prev_looking_into_eyes_vec is None:
                #         face_orientation_in_head_filtered = face_orientation_in_head_np
                #     else:
                #         face_orientation_in_head_filtered = self._prev_looking_into_eyes_vec * (1 - self._looking_into_eyes_vec_filter_co) + face_orientation_in_head_np * self._looking_into_eyes_vec_filter_co
                #         face_orientation_in_head_filtered = face_orientation_in_head_filtered / np.linalg.norm(face_orientation_in_head_filtered)
                #     # spot_on = (np.dot(EE_direction_in_head_np, face_orientation_in_head_filtered) > self._facing_angle_thres)
                #     spot_on = True
                #     self._looking_into_eyes_filter.update(spot_on)
                #     self._prev_looking_into_eyes_vec = face_orientation_in_head_filtered
                if curr_left_pointing or curr_right_pointing:
                    # if prev_pointing == 'Both' or prev_pointing != ('Left' if curr_left_pointing else 'Right')
                    try:
                        left_hand_trans = self._tfBuffer.lookup_transform('panda_link0', 'person_left_hand', rospy.Time())
                        right_hand_trans = self._tfBuffer.lookup_transform('panda_link0', 'person_right_hand', rospy.Time())
                        mid_trans = self._tfBuffer.lookup_transform("panda_link0", "person_spine_mid", rospy.Time())
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        print(f"Cannot look up transform between panda_link0 and person_spine_mid")
                        return
                    pointing_hand_trans = left_hand_trans if curr_left_pointing else right_hand_trans
                    self._looking_into_eyes_filter.update(pointing_hand_trans.transform.translation.z >= mid_trans.transform.translation.z)
                else:
                    self._looking_into_eyes_filter.update(True)

            look_into_eyes_state_change = prev_look_into_eyes != self._looking_into_eyes_filter.get_val()
            if look_into_eyes_state_change:
                # looking_into_eyes = self._prev_looking_into_eyes
                self._looking_into_eyes_changed_since = rospy.get_time()
                mediapipe_facing_msg = facing_EE()
                mediapipe_facing_msg.is_facing.data = self._looking_into_eyes_filter.get_val()
                self._pipe_facing_publisher.publish(mediapipe_facing_msg)


if __name__ == '__main__':
    mpr = MediaPipeReader()
    while not rospy.is_shutdown():
        try:
            mpr.read_data_and_publish()
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            print("Exit because of keyboard interruption")
            break

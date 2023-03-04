#!/usr/bin/env python3.8
## Try with Panda Robot

import json
from typing import Dict, List, Tuple

import rospy
import numpy as np
import numpy.linalg as linalg
import quaternion
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
from stargazer_msgs.msg import hand_gesture, facing_EE
import scipy.optimize
from generate_trajectories import get_orbit_path_func, get_orbit_directions_func
from stargazer_msgs.msg import prop_motion
import number_works
from kinect_udp_forward import UdpForward
import random
# from panda_robot import PandaArm

class HandVarianceEstimator(object):
    def __init__(self, estimate_span: float) -> None:    
        self._SPAN = estimate_span # in seconds
        self._data = []
        self._enough_data = False

    def add_new_data(self, incoming: List[np.ndarray]): 
        '''
        Add new hand position data, remove old points, recompute the mean and variance
        '''
        rightnow = rospy.Time.now()
        for point in incoming:
            self._data.append((rightnow, point))

        self.__remove_old_data(rightnow=rightnow, span=self._SPAN)
        

    def estimate(self) -> Tuple[float, float]:
        '''
        Compute hand position mean and variance
        '''
        ## a 'Gaussian sphere'
        all_points = np.vstack(tuple([dp[1] for dp in self._data]))
        # print(f"Data so far is {all_points}")
        center = np.mean(all_points, axis=0)
        all_points_centered = all_points - np.tile(center, (len(all_points), 1))
        #return center, max(0.1, np.std(np.linalg.norm(all_points_centered, axis=1))) # define some minimum object size
        return center, (np.clip(np.mean(np.linalg.norm(all_points_centered, axis=1)), 0.1, 0.25) if self._enough_data else 0.15) # define some minimum object size
        

    def __remove_old_data(self, rightnow, span: float) -> None:
        for ind in range(0, len(self._data)):
            # print(f"CHecking old data at {self._data[ind][0].secs} against {rightnow.secs}")
            if self._data[ind][0].secs < rightnow.secs - span: # so this data point is old
                ind += 1
            else:
                break
        if ind > 0 and len(self._data) > ind:
            ## Remove index 0 -> ind - 1
            # print(f"Removed {ind + 1} points")
            self._data = self._data[ind:]
            if not self._enough_data:
                self._enough_data = True

class PandaFollowHand(object):

    def __init__(self) -> None:
        rospy.init_node("panda_follow_hand") # initialise ros node
        # self._arm = PandaArm()
        # self._est_rate = rospy.Rate(1 / 1.0) # update hand pose estimation every 1 second
        self._lookup_rate = rospy.Rate(30.0) 
        #self._since_last_est = 0
        #self._EST_RATE = 1.0
        #self._est_cycle = 30.0 / self._EST_RATE
        self._plan_cycle = 30.0 / 5
        self._since_last_plan = 0
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._ROBOT_BASE = 'panda_link0'
        self._FOV = 60.0 / 180 * np.pi
        self._ASPECT_RATIO = 16.0 / 9.0
        self._TABLE_POINT = [0.5, 0, 0.1]
        self._TABLE_NORMAL = [0, 0, 1.0]
        self._TABLE_X_RANGE = [0.8, 1.4] # TODO: update this 
        self._TABLE_Y_RANGE = [-0.6, 0.6] # TODO: update this
        # self._action_pub = rospy.Publisher('robot_action/goto', geometry_msgs.msg.Transform, queue_size=1)
        self._action_pub = rospy.Publisher('target_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        # angle_diff_thres = np.cos(np.pi / 12) # 15 dgree cone
        self._hand_sub = rospy.Subscriber('mediapipe_pointing', hand_gesture, self.__mediapipe_pointing_cb, queue_size=1)
        self._face_sub = rospy.Subscriber('mediapipe_facing', facing_EE, self.__mediapipe_facing_cb, queue_size=1)
        # self._normal_sub = rospy.Subscriber('hand_normal', geometry_msgs.msg.Vector3Stamped, self.__hand_normal_updated, queue_size=10) # large queue size necessary?
        self._speech_sub = rospy.Subscriber('hand_speech_signal', std_msgs.msg.String, self.__speech_cb, queue_size=10)
        self._prop_sub = rospy.Subscriber('prop_motion', prop_motion, self.__prop_cb, queue_size=10)

        self._hand_var_est = HandVarianceEstimator(5.0)
        self._hand_var = 0.15
        # self._hand_mean = 
        self._speech_sig = 'normal' ## 'overhead', 'close-up', 'normal'
        self._recent_prop_motion = '' ## 'rotate', 'zoom', 'pan'
        self._angle_mode = "level" ## "level", "top"
        self._dist_mode = "medium" ## "medium", "close"
        self._trajectory_func_pan = None
        self._trajectory_func_orbit = None
        self._anim_duration = {
            'pan': 10.0,
            'orbit': 10.0,
            'cycle': 0.0
            # 'closeup': 10.0,
            # 'overhead': 30.0 
        }
        self._last_anim_ts = -1
        self._start_ts = -1
        self._angle_variation_func = None
        self._is_orbiting = False
        self._left_hand_relaxed = number_works.BinaryFilter(init_val=False, filter_co=0.2)
        self._right_hand_relaxed = number_works.BinaryFilter(init_val=False, filter_co=0.2)
        self._zoom_udp = UdpForward("172.21.18.117", 12345)

        polar_min = [-np.pi / 2 * 0.9, -np.pi / 12, 0.36] # theta, phi, R
        polar_max = [np.pi / 2 * 0.9, np.pi / 2 * 0.8, 0.66]
        self._polar_min = polar_min
        self._polar_max = polar_max

        # cam_to_EE_transform = self._tfBuffer.lookup_transform('panda_link8','pixel6pro', rospy.Time())
        # self._cam_to_EE_translation = np.array([cam_to_EE_transform.transform.translation.x,
        #                                         cam_to_EE_transform.transform.translation.y,
        #                                         cam_to_EE_transform.transform.translation.z])
        # rospy.loginfo(f"Translation is {self._cam_to_EE_translation}")
        #self._EE_to_cam_transform = self._tfBuffer.lookup_transform('pixel6pro', 'panda_link8', rospy.Time())

        # self._arm = PandaArm()
        # self._JOINT_LIMIT_GUARD_BUFFER = 5 * np.pi / 180.0
        # self._joint_limits = [(lower_upper['lower'] + self._JOINT_LIMIT_GUARD_BUFFER, lower_upper['upper'] - self._JOINT_LIMIT_GUARD_BUFFER) 
        #                         for lower_upper in self._arm.joint_limits()]

        #print(f"Grid shape {cost_grid_x}")
        self._prev_cam = None #TODO: maybe change to the reference pose
        self._prev_look_vec = None
        self._prev_cam_zoom = 1.0
        self._prev_shoulder_dir_filter = number_works.Vector3Filter(None, filter_co=0.25, normalize=True)

        ### MediaPipe stuffs ###
        # hand gestures
        self._prev_pointing = '' ## can be 'Left', 'Right', ''
        self._recent_pointing_trace = []
        self._RECENT_POINTING_TRACE_LEN = 5
        self._POINTING_MOVE_MIN = 0.05
        self._pointing_detected = '' ## can be 'Left', 'Right', ''

        # face orientation
        self._looking_into_eyes = False
        self._was_looking_into_eyes = False

        # orientation change
        self._neutral_orientation = np.array([1, 0, 0])
        self._other_orientation = [np.array([0.707, 0.707, 0]), np.array([0.707, -0.707, 0])]
        self._orientation_ind = 0
        self._vary_orientation_prob = 0.01
        self._vary_orientation = False


    def mat_from_RT(self, R, T):
        mat = np.zeros((4, 4))
        mat[:3, :3] = R.copy()
        mat[:3, 3] = T.flatten()
        mat[3, :] = np.array([0, 0, 0, 1])
        return mat

    @staticmethod
    def project_vec_to_plane(v: np.ndarray, n: np.ndarray, normalize = True) -> np.ndarray:
        # result = v - np.dot(v, n) / (np.linalg.norm(n) ** 2) * n
        # return result / np.linalg.norm(result) if normalize else result
        return number_works.project_vec_to_plane(v, n , normalize)

    # def __hand_normal_updated(self, hand_normal_vec: geometry_msgs.msg.Vector3Stamped) -> None:
    #     #self._hand_normal_lock.acquire()
    #     self._hand_normal = np.array([hand_normal_vec.vector.x, hand_normal_vec.vector.y, hand_normal_vec.vector.z])
    #     #self._hand_normal_lock.release()
    #     ## Maybe reset this after a while?

    def __speech_cb(self, speech_msg: std_msgs.msg.String) -> None:
        self._speech_sig = speech_msg.data
        if self._speech_sig == 'overhead':
            self._angle_mode = 'top'
            rospy.loginfo("Setting angle mode to top")
            #self._last_top_mode_ts = rospy.Time.now().to_sec()
        elif self._speech_sig == 'close-up':
            self._dist_mode = 'close'
            self.__send_cam_zoom(2.0)
            # rospy.loginfo(f"Sending zoom level {self._prev_cam_zoom}")
            rospy.loginfo("Setting dist mode to closeup")
            #self._last_close_mode_ts = rospy.Time.now().to_sec()

    def __prop_cb(self, prop_motion_msg: prop_motion) -> None:
        data = prop_motion_msg.motion.data
        if data in ["top", "level"]: ## Other modes, here just for prototyping
            self._angle_mode = data
            rospy.loginfo(f"Setting mode to be {data} from prop input")
        elif data in ["close", "medium"]:
            self._dist_mode = data
            if data == 'close':
                self.__send_cam_zoom(2.0)
            else:
                self.__send_cam_zoom(1.0)
            rospy.loginfo(f"Setting mode to be {data} from prop input")
        elif data == 'orbit':
            self._is_orbiting = True
            rospy.loginfo(f"Setting mode to be {data} from prop input")

        # elif data == 'left':
        #     self._prop_pointing_left = not self._prop_pointing_left
        #     rospy.loginfo(f"Setting left pointing to be {self._prop_pointing_left} from prop input")
        # elif data == 'right':
        #     self._prop_pointing_right = not self._prop_pointing_right
        #     rospy.loginfo(f"Setting left pointing to be {self._prop_pointing_right} from prop input")
        # else:
        #     self._recent_prop_motion = data
        # pass

    def __mediapipe_facing_cb(self, facing_msg: facing_EE) -> None:
        self._looking_into_eyes = facing_msg.is_facing.data
        # if facing_msg.is_facing.data:
        #     self._looking_into_eyes_started_at = rospy.get_time()
        # else:
        #     self._looking_into_eyes_started_at = -1.0

    def __mediapipe_pointing_cb(self, media_pipe_msg: hand_gesture) -> None:
        #-------- hand pointing -----------
        self._pointing_detected = media_pipe_msg.handedness.data

    def __send_cam_zoom(self, cam_zoom_val: float) -> None:
        self._zoom_udp.forward_str(json.dumps({"camera_zoom": cam_zoom_val}))

    def __send_cam_pointing(self, pointing_state: str) -> None:
        self._zoom_udp.forward_str(json.dumps({"pointing": pointing_state}))

    def __send_cam_robot_state(self, robot_state: str) -> None:
        self._zoom_udp.forward_str(json.dumps({"state": robot_state}))

    def __send_cam_status_disp(self, pointing: str, robot_state: str) -> None:
        self.__send_cam_pointing(pointing_state=pointing)
        self.__send_cam_robot_state(robot_state=robot_state)
        


    # @staticmethod 
    # def rotate_shoulder_line(shoulder_ori: np.ndarray, side: str, rotate_by: float) -> np.ndarray:
    #     '''
    #     shoulder should be from right to left
    #     '''
    #     rotation_ax = np.array([0, 0, 1]) # rotate around the gravity
    #     if side == 'right':
    #         rotation_ax_ang = rotation_ax * (-rotate_by)
    #     else:
    #         rotation_ax_ang = rotation_ax * (rotate_by - np.pi)
    #     rotation_q = quaternion.from_rotation_vector(rotation_ax_ang)
    #     shoulder_ori_rotated = quaternion.as_vector_part(rotation_q * quaternion.from_vector_part(shoulder_ori) * rotation_q.conjugate()).flatten()
    #     return shoulder_ori_rotated

    # @staticmethod
    # def obj_in_fov_center(p_cam_frame: np.ndarray, h_fov: float, aspect_ratio: float, portion: float) -> bool:
    #     '''
    #     Check if an object is in the center (according to the `portion` param provided) of the camera frame
    #     '''
    #     px, py, pz = p_cam_frame[0], p_cam_frame[1], p_cam_frame[2]
    #     return np.abs(px / pz) / np.tan(h_fov / 2.0) < portion and np.abs(py / pz) / np.tan(h_fov / 2.0) / aspect_ratio < portion

    # def compute_cost_look_at_hand(self, hands_pos: List[np.ndarray], wrists_pos: List[np.ndarray], hand_var: np.ndarray, head_pos: np.ndarray, pointing: str, facing:bool, grid_xx: np.ndarray, grid_yy: np.ndarray, grid_zz: np.ndarray,
    #                             prev_cam: np.ndarray, prev_look: np.ndarray, ortho_to: np.ndarray, ori_smooth_co: float, ori_co: float, zoom_co: float, pitch_co: float) -> Tuple[np.ndarray, np.ndarray]:
    #                             #prev_cam: np.ndarray, prev_look: np.ndarray, align_with: np.ndarray, ori_smooth_co: float, ori_co: float, zoom_co: float) -> Tuple[np.ndarray, np.ndarray]:
    #     '''
    #     Find the optimal camera position based on some cost function 
    #     Do a grid search within the bounds to optimize
    #     '''
    #     pitch_goal = -1
    #     spread = 2 * hand_var
    #     if facing:
    #         ## Face mode takes priority
    #         # Look at the center between the head the the hands (center of a triangle ?)
    #         # triangle = np.vstack((np.array(hands_pos), head_pos))
    #         focus = head_pos
    #         #triangle_centered = triangle - np.tile(focus, (len(triangle), 1))
    #         spread = 0.12
    #         pitch_goal = 0
    #     elif len(hands_pos) == 2:
    #         if pointing == 'Left':
    #             focus = hands_pos[0] + (hands_pos[0] - wrists_pos[0]) * 1.2
    #             spread = 0.1
    #             #print(f"Zooming onto {pointing}")
    #         elif pointing == 'Right':
    #             focus = hands_pos[1] + (hands_pos[1] - wrists_pos[1]) * 1.2
    #             spread = 0.1
    #             #print(f"Zooming onto {pointing}")
    #         else:
    #             focus = (hands_pos[0] + hands_pos[1]) / 2
    #     else:
    #         focus = hands_pos[0]

    #     dist_smoothing_term = ((grid_xx - prev_cam[0]) ** 2 + (grid_yy - prev_cam[1]) ** 2 + (grid_zz - prev_cam[2]) ** 2) * 20 ## Quadratic
    #     # dist_smoothing_term = np.sqrt((grid_xx - prev_cam[0]) ** 2 + (grid_yy - prev_cam[1]) ** 2 + (grid_zz - prev_cam[2]) ** 2) * 2.0 ## L1

    #     centered_xx, centered_yy, centered_zz = focus[0] - grid_xx, focus[1] - grid_yy, focus[2] -  grid_zz
    #     centered_norms = np.sqrt(centered_xx ** 2 + centered_yy ** 2 + centered_zz ** 2)
    #     look_xx, look_yy, look_zz = centered_xx / centered_norms, centered_yy / centered_norms, centered_zz / centered_norms
    #     look_floor_norms = np.sqrt(look_xx ** 2 + look_yy ** 2)
    #     look_floor_xx, look_floor_yy = look_xx / look_floor_norms, look_yy / look_floor_norms
    #     # orientation_term = 1 - (look_xx * align_with[0] + look_yy * align_with[1] + look_zz * align_with[2]) ## Align with a vector
    #     # orientation_term = np.abs(look_xx * ortho_to[0] + look_yy * ortho_to[1] + look_zz * ortho_to[2]) ## Orthogonal to a vector
    #     ## Use absolute diff rather than cosine for a sharper gradient
    #     # orientation_term = np.abs(np.arccos(look_floor_xx * ortho_to[0] + look_floor_yy * ortho_to[1]) - np.pi / 2) ## Orthogonal to a vector on the x-y plane
    #     orientation_term = (np.arccos(look_floor_xx * ortho_to[0] + look_floor_yy * ortho_to[1]) - np.pi / 2) ** 2 ## Orthogonal to a vector on the x-y plane (quadratic)
    #     look_at_D = spread * 3.0 / 2.0 / np.tan(self._FOV / 2)
    #     zoom_term = np.abs(centered_norms / look_at_D - 1)
    #     orientation_smoothing_term = 1 - (look_xx * prev_look[0] + look_yy * prev_look[1] + look_zz * prev_look[2])
    #     # print(f"Shape of grid_xx matrix {self._box_xx.shape}")
    #     pitch_term = np.abs(pitch_goal - look_zz / np.sqrt(look_xx ** 2 + look_yy ** 2)) if pitch_goal >= 0 else 0
    #     cost = dist_smoothing_term + ori_smooth_co * orientation_smoothing_term + zoom_term * zoom_co + orientation_term * ori_co + pitch_term * pitch_co
    #     # print(f"Shape of cost matrix {cost.shape}")
    #     ind_min = np.unravel_index(np.argmin(cost, axis=None), cost.shape)
    #     look_from_pos = np.array([grid_xx[ind_min], grid_yy[ind_min], grid_zz[ind_min]])
    #     look_vec = focus - look_from_pos
    #     look_vec = look_vec / np.linalg.norm(look_vec)
    #     return look_from_pos, look_vec

    # @staticmethod
    # def __EE_pose_level_constraint(joints: np.ndarray, EE_to_cam_trans_msg: geometry_msgs.msg.TransformStamped, robot: PandaArm) -> float:
    #     EE_pos, rot_quat = robot.forward_kinematics(joints)
    #     EE_pos = EE_pos.flatten()
    #     cam_mat = PandaFollowHand.__EE_pose_to_camera_pose(EE_pos=EE_pos, EE_rot_quat=rot_quat, EE_to_cam_trans_msg=EE_to_cam_trans_msg)
    #     return np.dot((cam_mat @ np.array([[1], [0], [0], [1]]))[:3,:].flatten(), np.array([0, 0, 1]))

    # @staticmethod
    # def __EE_pose_to_camera_pose(EE_pos: np.ndarray, EE_rot_quat: quaternion.quaternion, EE_to_cam_trans_msg: geometry_msgs.msg.TransformStamped) -> np.ndarray:
    #     transform_from_msg = EE_to_cam_trans_msg.transform
    #     EE_to_cam_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix([transform_from_msg.translation.x, transform_from_msg.translation.y, transform_from_msg.translation.z]), 
    #                                                                         tf_conversions.transformations.quaternion_matrix([transform_from_msg.rotation.x, transform_from_msg.rotation.y, transform_from_msg.rotation.z, transform_from_msg.rotation.w]))
    #     cam_to_EE_mat = tf_conversions.transformations.inverse_matrix(EE_to_cam_mat)
    #     EE_to_base_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix(EE_pos), 
    #                                                                         tf_conversions.transformations.quaternion_matrix([EE_rot_quat.x, EE_rot_quat.y, EE_rot_quat.z, EE_rot_quat.w]))
    #     cam_to_base_mat = tf_conversions.transformations.concatenate_matrices(EE_to_base_mat, cam_to_EE_mat)
    #     return cam_to_base_mat


    # @staticmethod
    # def __cost_look_at_hand_joints(joints: np.ndarray, focus: np.ndarray, desired_d: float, EE_to_cam_trans_msg: geometry_msgs.msg.TransformStamped, robot: PandaArm,
    #                             prev_cam: np.ndarray, desired_dir: np.ndarray, desired_pitch: float, 
    #                             disp_smoothing_co: float, lookat_co: float, ori_co: float, zoom_co: float, pitch_co: float) -> float:
    #     EE_pos, rot_quat = robot.forward_kinematics(joints)
    #     EE_pos = EE_pos.flatten()
    #     cam_mat = PandaFollowHand.__EE_pose_to_camera_pose(EE_pos=EE_pos, EE_rot_quat=rot_quat, EE_to_cam_trans_msg=EE_to_cam_trans_msg)
    #     cam_pos = tf_conversions.transformations.translation_from_matrix(cam_mat)
    #     cam_look = (cam_mat @ np.array([[0], [0], [1], [1]]))[:3,:].flatten()
        
    #     disp_smoothing_term = np.linalg.norm(cam_pos - prev_cam) ** 2
    #     ideal_look_vec = focus - cam_pos
    #     ideal_look_proj_cam_look = np.dot(cam_look, ideal_look_vec) * cam_look
    #     look_vec_floor = PandaFollowHand.project_vec_to_plane(cam_look, np.array([0, 0, 1]), normalize=True)
    #     # look_vec_wall = PandaFollowHand.project_vec_to_plane(look_vec, np.array([0, 1, 0]))
    #     ori_term = np.linalg.norm(look_vec_floor - desired_dir) ** 2
    #     pitch_term = 0 if desired_pitch <= 0 else (np.dot(np.array([0, 0, -1]), cam_look / np.linalg.norm(cam_look)) -  np.cos(np.pi / 2 - desired_pitch)) ** 2
    #     desired_dist_term = (np.linalg.norm(ideal_look_proj_cam_look)- desired_d) ** 2 
    #     ideal_look_term = np.linalg.norm(ideal_look_vec / np.linalg.norm(ideal_look_vec) - cam_look) ** 2
    #     # camera_level_term = np.dot((cam_mat @ np.array([[1], [0], [0], [1]]))[:3,:].flatten(), np.array([0, 0, 1])) ** 2
    #     return (disp_smoothing_co * disp_smoothing_term + ori_co * ori_term + desired_dist_term * zoom_co + 
    #             pitch_term * pitch_co + lookat_co * ideal_look_term)

    # def compute_cost_look_at_hand_joints(self, hands_pos: List[np.ndarray], wrists_pos: List[np.ndarray], hand_var: np.ndarray, head_pos: np.ndarray, 
    #                             EE_to_cam_trans: geometry_msgs.msg.TransformStamped, robot: PandaArm,
    #                             pointing: str, facing:bool, prev_cam: np.ndarray, desired_dir: np.ndarray, desired_pitch: float, 
    #                             disp_smooth_co: float, ori_co: float, zoom_co: float, pitch_co: float, lookat_co: float) -> Tuple[np.ndarray, np.ndarray]:
    #     '''
    #     Use non-linear optimization to find the (local) minima
    #     '''
    #     # pitch_goal = -1
    #     spread = 2 * hand_var
    #     now = rospy.Time.now().to_sec()
    #     if facing:
    #         ## Face mode takes priority
    #         focus = head_pos
    #         spread = 0.18
    #         desired_pitch = 0
    #     elif len(hands_pos) == 2:
    #         if pointing == 'Left' or pointing == 'Right':
    #             # Check if the finger is drawing a continuous line
    #             ori_co = 0 # Do not enforce look direction unless it's panning
    #             if self._prev_pointing == pointing:
    #                 self._recent_pointing_trace.append(hands_pos[0] if pointing == 'left' else hands_pos[1])
    #                 if len(self._recent_pointing_trace) > self._RECENT_POINTING_TRACE_LEN:
    #                     self._recent_pointing_trace.pop(0)
    #                 if np.linalg.norm(self._recent_pointing_trace[-1] - self._recent_pointing_trace[0]) > self._POINTING_MOVE_MIN:
    #                     desired_dir = np.array([1, 0, 0]) # if moving, then look ahead
    #                     ori_co = 2.0
    #             else: 
    #                 self._recent_pointing_trace.clear()
    #                 self._recent_pointing_trace.append(hands_pos[0] if pointing == 'left' else hands_pos[1])
                    
    #             spread = 0.18
                
    #             if pointing == 'Left':
    #                 #focus = hands_pos[0] + (hands_pos[0] - wrists_pos[0]) * 1.2
    #                 focus = number_works.intersect_vec_plane(self._TABLE_POINT, self._TABLE_NORMAL, hands_pos[0], number_works.normalize_vec(hands_pos[0] - wrists_pos[0]))
    #                 in_range = focus is not None and focus[0] < self._TABLE_X_RANGE[1] and focus[0] > self._TABLE_X_RANGE[0] and focus[1] < self._TABLE_Y_RANGE[1] and focus[1] > self._TABLE_Y_RANGE[0]
    #                 focus = focus if in_range else hands_pos[0] + (hands_pos[0] - wrists_pos[0]) * 0.5
    #             else:
    #                 #focus = hands_pos[1] + (hands_pos[1] - wrists_pos[1]) * 1.2
    #                 focus = number_works.intersect_vec_plane(self._TABLE_POINT, self._TABLE_NORMAL, hands_pos[1], number_works.normalize_vec(hands_pos[1] - wrists_pos[1]))
    #                 in_range = focus is not None and focus[0] < self._TABLE_X_RANGE[1] and focus[0] > self._TABLE_X_RANGE[0] and focus[1] < self._TABLE_Y_RANGE[1] and focus[1] > self._TABLE_Y_RANGE[0]
    #                 focus = focus if in_range else hands_pos[1] + (hands_pos[1] - wrists_pos[1]) * 0.5
                
    #             #rospy.loginfo(f"Ortho_to set to be {ortho_to}")
    #         elif pointing == 'Both':
    #             spread = 0.18
    #             focus = (hands_pos[0] + hands_pos[1]) / 2
    #             if not self._is_orbiting:
    #                 self._is_orbiting = True
    #         else:
    #             focus = (hands_pos[0] + hands_pos[1]) / 2
    #             # focus = hands_mean
    #     else:
    #         focus = hands_pos[0]

    #     self._prev_pointing = pointing
    #     look_at_D = spread * 2.0 / 2.0 / np.tan(self._FOV / 2)
    #     ## Use previous camera position as initial condition
    #     if not self._is_orbiting:
    #         # init_theta = np.arctan(prev_cam[1] / prev_cam[0])
    #         # init_phi = np.arctan((prev_cam[2] - 0.33) / (prev_cam[1] / np.sin(init_theta)))
    #         # init_r = (prev_cam[2] - 0.33) / np.sin(init_phi)
    #         initial_joints = self._arm.angles()

    #         obj_args = (focus, look_at_D, EE_to_cam_trans, robot, prev_cam, desired_dir, desired_pitch, disp_smooth_co, ori_co, zoom_co, pitch_co, lookat_co)
    #         #op_theta_phi_r_res = scipy.optimize.minimize(PandaFollowHand.__cost_look_at_hand_op_wzoom_obj, np.array([init_theta, init_phi, init_r, prev_cam_zoom]), args=obj_args, 
    #         look_joints = scipy.optimize.minimize(PandaFollowHand.__cost_look_at_hand_joints, initial_joints, args=obj_args, 
    #                                                 # method='L-BFGS-B',
    #                                                 method="SLSQP",
    #                                                 #bounds=((self._polar_min[0], self._polar_max[0]), (self._polar_min[1], self._polar_max[1]), (self._polar_min[2], self._polar_max[2]), (1.0, 2.0)))
    #                                                 constraints={'type': 'eq', 'fun': PandaFollowHand.__EE_pose_level_constraint, 'args': (EE_to_cam_trans, robot)},
    #                                                 bounds=self._joint_limits).x

    #         # look_from_pos = PandaFollowHand.__polar_to_cartesian(op_theta_phi_r_res.x[0:3])
    #         EE_pos, EE_rot_quat = robot.forward_kinematics(look_joints)
    #         cam_mat = PandaFollowHand.__EE_pose_to_camera_pose(EE_pos=EE_pos.flatten(), EE_rot_quat=EE_rot_quat, EE_to_cam_trans_msg=EE_to_cam_trans)
    #         look_from_pos = tf_conversions.transformations.translation_from_matrix(cam_mat)
    #         look_vec = (cam_mat @ np.array([[0], [0], [1], [1]]))[:3,:].flatten()
    #     else:
    #         # focus = np.array([0.95, 0, 0])
    #         if self._trajectory_func_orbit is None:
    #             self._last_anim_ts = now
    #             # init_dir = focus - prev_cam
    #             # init_dir = init_dir / np.linalg.norm(init_dir)
    #             self._trajectory_func_orbit = get_orbit_path_func(center=focus, radius = 0.6, pitch=np.pi / 6, angle=np.pi / 4, start = prev_cam, start_time=0.2)
    #         orbiting_prog = (now - self._last_anim_ts) / self._anim_duration["orbit"]
    #         look_from_pos = self._trajectory_func_orbit(np.clip(orbiting_prog, 0.0, 1.0))
    #         if orbiting_prog > 1.0:
    #             self._is_orbiting = False
    #             self._trajectory_func_orbit = None
    #         look_vec = focus - look_from_pos
    #         look_vec = look_vec / np.linalg.norm(look_vec)
    #     # return look_from_pos, look_vec, op_theta_phi_r_res.x[3]
    #     return look_from_pos, look_vec

    @staticmethod
    def __polar_to_cartesian(theta_phi_r: np.ndarray) -> np.ndarray:
        r = theta_phi_r[2]
        sin_theta = np.sin(theta_phi_r[0])
        cos_theta = np.cos(theta_phi_r[0])
        sin_phi = np.sin(theta_phi_r[1])
        cos_phi = np.cos(theta_phi_r[1])
        return np.array([r * cos_phi * cos_theta, r * cos_phi * sin_theta, r * sin_phi + 0.333]) 
        

    @staticmethod
    def __cost_look_at_hand_op_obj(theta_phi_r: np.ndarray, 
                                #cam_to_EE_translation: np.ndarray,
                                focus: np.ndarray, desired_d: float, 
                                prev_cam: np.ndarray, desired_dir: np.ndarray, desired_pitch: float, 
                                disp_smoothing_co: float, ori_co: float, zoom_co: float, pitch_co: float) -> float:
        x = PandaFollowHand.__polar_to_cartesian(theta_phi_r=theta_phi_r)
        #vx = x + cam_to_EE_translation
        disp_smoothing_term = np.linalg.norm(x - prev_cam) ** 2
        look_vec = focus - x
        look_vec_floor = PandaFollowHand.project_vec_to_plane(look_vec, np.array([0, 0, 1]), normalize=True)
        # look_vec_wall = PandaFollowHand.project_vec_to_plane(look_vec, np.array([0, 1, 0]))
        ori_term = np.linalg.norm(look_vec_floor - desired_dir) ** 2
        pitch_term = (np.dot(np.array([0, 0, -1]), look_vec / np.linalg.norm(look_vec)) -  np.cos(np.pi / 2 - desired_pitch)) ** 2
        desired_dist_term = (np.linalg.norm(look_vec)- desired_d) ** 2 
        return disp_smoothing_co * disp_smoothing_term + ori_co * ori_term + desired_dist_term * zoom_co + pitch_term * pitch_co

    # @staticmethod
    # def __check_hand_relaxed(hand_left: np.ndarray, hand_right: np.ndarray, shoulder_left: np.ndarray, shoulder_right: np.ndarray) -> Tuple[bool, bool]:
    #     angle_threshold = np.cos(np.pi / 18)
    #     up = np.array([0, 0, 1])
    #     shoulder_hand_left = number_works.project_vec_to_plane(hand_left - shoulder_left, up, normalize=True)
    #     shoulder_hand_right = number_works.project_vec_to_plane(hand_right - shoulder_right, up, normalize=True)
    #     shoudler_left_right = number_works.project_vec_to_plane(shoulder_right -  shoulder_left, up, normalize=True)
    #     return np.dot(shoulder_hand_left, -shoudler_left_right) > angle_threshold, np.dot(shoulder_hand_right, shoudler_left_right) > angle_threshold

    def __check_hand_behind(hand_left: np.ndarray, hand_right:np.ndarray, hip: np.ndarray) -> Tuple[bool, bool]:
        return hand_left[0] > hip[0], hand_right[0] > hip[0]

    def compute_cost_look_at_hand_op(self, hands_pos: List[np.ndarray], hands_mean: np.ndarray, wrists_pos: List[np.ndarray], hand_var: np.ndarray, head_pos: np.ndarray, 
                                pointing: str, facing:bool, prev_cam: np.ndarray, desired_dir: np.ndarray, desired_pitch: float, 
                                disp_smooth_co: float, ori_co: float, zoom_co: float, pitch_co: float) -> Tuple[np.ndarray, np.ndarray]:
        '''
        Use non-linear optimization to find the (local) minimal
        '''
        # pitch_goal = -1
        spread = 2 * hand_var
        now = rospy.Time.now().to_sec()
        if facing:
            ## Face mode takes priority
            focus = head_pos - 0.17
            spread = 0.2
            desired_pitch = 0
            pitch_co = 2.0
            desired_dir = [1, 0, 0]
            ori_co = 1.0
            #self._both_pointing_counter = 0
        elif len(hands_pos) == 2:
            if pointing == 'Left' or pointing == 'Right':
                # Check if the finger is drawing a continuous line
                ori_co = 0 # Do not enforce look direction unless it's panning
                if self._prev_pointing == pointing:
                    self._recent_pointing_trace.append(hands_pos[0] if pointing == 'left' else hands_pos[1])
                    if len(self._recent_pointing_trace) > self._RECENT_POINTING_TRACE_LEN:
                        self._recent_pointing_trace.pop(0)
                    if np.linalg.norm(self._recent_pointing_trace[-1] - self._recent_pointing_trace[0]) > self._POINTING_MOVE_MIN:
                        desired_dir = np.array([1, 0, 0]) # if moving, then look ahead
                        ori_co = 1.0
                        # disp_smooth_co = 4.0
                else: 
                    self._recent_pointing_trace.clear()
                    self._recent_pointing_trace.append(hands_pos[0] if pointing == 'left' else hands_pos[1])
                    
                spread = 0.12
                #self._both_pointing_counter = 0
                
                if pointing == 'Left':
                    # focus = hands_pos[0] + (hands_pos[0] - wrists_pos[0]) * 1.2
                    left_tip_estimate = hands_pos[0] + (hands_pos[0] - wrists_pos[0]) * 1.0
                    # left_tip_estimate = hands_pos[0]
                    # focus = number_works.intersect_vec_plane(self._TABLE_POINT, self._TABLE_NORMAL, hands_pos[0], number_works.normalize_vec(left_tip_estimate - head_pos))
                    # in_range = focus is not None and focus[0] < self._TABLE_X_RANGE[1] and focus[0] > self._TABLE_X_RANGE[0] and focus[1] < self._TABLE_Y_RANGE[1] and focus[1] > self._TABLE_Y_RANGE[0]
                    focus = left_tip_estimate
                else:
                    #focus = hands_pos[1] + (hands_pos[1] - wrists_pos[1]) * 1.2
                    right_tip_estimate = hands_pos[1] + (hands_pos[1] - wrists_pos[1]) * 1.0
                    # right_tip_estimate = hands_pos[1]
                    # focus = number_works.intersect_vec_plane(self._TABLE_POINT, self._TABLE_NORMAL, hands_pos[1], number_works.normalize_vec(right_tip_estimate - head_pos))
                    # in_range = focus is not None and focus[0] < self._TABLE_X_RANGE[1] and focus[0] > self._TABLE_X_RANGE[0] and focus[1] < self._TABLE_Y_RANGE[1] and focus[1] > self._TABLE_Y_RANGE[0]
                    focus = right_tip_estimate
                
                #rospy.loginfo(f"Ortho_to set to be {ortho_to}")
            elif pointing == 'Both':
                spread = 0.18
                focus = (hands_pos[0] + hands_pos[1]) / 2
                if not self._is_orbiting:
                    self._is_orbiting = True
            else:
                focus = (hands_pos[0] + hands_pos[1]) / 2
                #self._both_pointing_counter = 0
        else:
            focus = hands_pos[0]
            #self._both_pointing_counter = 0

        self._prev_pointing = pointing
        #rospy.loginfo(f"Previous pointing is {self._prev_pointing}")

        look_at_D = spread * 2.0 / 2.0 / np.tan(self._FOV / 2)
        ## Use previous camera position as initial condition
        if not self._is_orbiting:
            init_theta = np.arctan(prev_cam[1] / prev_cam[0])
            init_phi = np.arctan((prev_cam[2] - 0.333) / (prev_cam[1] / np.sin(init_theta)))
            init_r = (prev_cam[2] - 0.333) / np.sin(init_phi)

            obj_args = (focus, look_at_D, prev_cam, desired_dir, desired_pitch, disp_smooth_co, ori_co, zoom_co, pitch_co)
            #op_theta_phi_r_res = scipy.optimize.minimize(PandaFollowHand.__cost_look_at_hand_op_wzoom_obj, np.array([init_theta, init_phi, init_r, prev_cam_zoom]), args=obj_args, 
            op_theta_phi_r_res = scipy.optimize.minimize(PandaFollowHand.__cost_look_at_hand_op_obj, np.array([init_theta, init_phi, init_r]), args=obj_args, 
                                                    method='L-BFGS-B', 
                                                    #bounds=((self._polar_min[0], self._polar_max[0]), (self._polar_min[1], self._polar_max[1]), (self._polar_min[2], self._polar_max[2]), (1.0, 2.0)))
                                                    bounds=((self._polar_min[0], self._polar_max[0]), (self._polar_min[1], self._polar_max[1]), (self._polar_min[2], self._polar_max[2])))

            # look_from_pos = PandaFollowHand.__polar_to_cartesian(op_theta_phi_r_res.x[0:3])
            look_from_pos = PandaFollowHand.__polar_to_cartesian(op_theta_phi_r_res.x)
            look_vec = focus - look_from_pos
            look_vec = look_vec / np.linalg.norm(look_vec)
        else:
            if self._trajectory_func_orbit is None:
                self._last_anim_ts = now
                orbit_offset = np.array([0, 0, -0.1])
                self._trajectory_func_orbit = get_orbit_path_func(center=focus + orbit_offset, radius = 0.6, pitch=np.pi / 9, angle=np.pi / 4, start = prev_cam, start_time=0.2)
            orbiting_prog = (now - self._last_anim_ts) / self._anim_duration["orbit"]
            look_from_pos, look_vec = self._trajectory_func_orbit(np.clip(orbiting_prog, 0.0, 1.0))
            if orbiting_prog > 1.0:
                self._is_orbiting = False
                self._trajectory_func_orbit = None
        # return look_from_pos, look_vec, op_theta_phi_r_res.x[3]
        return look_from_pos, look_vec

    def listen(self, hands_name, head_name, shoulders_name, spine_name) -> None: 
        '''
        Get hand pose in the robot base frame
        '''
        while not rospy.is_shutdown():

            # Add condition if mode == HAND_MODE
            no_hands = True
            hands_trans = []
            wrists_trans = []
            for hand_name, wrist_name in hands_name:
                try:
                    hands_trans.append(self._tfBuffer.lookup_transform(self._ROBOT_BASE, hand_name, rospy.Time()))
                    wrists_trans.append(self._tfBuffer.lookup_transform(self._ROBOT_BASE, wrist_name, rospy.Time()))
                    no_hands = False
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    # print(f"Cannot find hand {hand_name}")
                    # self._lookup_rate.sleep()
                    continue

            if no_hands:
                self._lookup_rate.sleep()
                continue

            shoulders_trans = []
            #no_shoulder = False
            try:
                cam_to_EE_transform = self._tfBuffer.lookup_transform('panda_link8','pixel6pro', rospy.Time())
                self._cam_to_EE_translation = np.array([cam_to_EE_transform.transform.translation.x,
                                                        cam_to_EE_transform.transform.translation.y,
                                                        cam_to_EE_transform.transform.translation.z])
                self._EE_to_cam_transform = self._tfBuffer.lookup_transform('pixel6pro', 'panda_link8', rospy.Time())

                for shoulder_name in shoulders_name:
                    shoulders_trans.append(self._tfBuffer.lookup_transform(self._ROBOT_BASE, shoulder_name, rospy.Time()))
                current_cam_to_base_trans = self._tfBuffer.lookup_transform(self._ROBOT_BASE, 'pixel6pro', rospy.Time())
                self._prev_cam = np.array([current_cam_to_base_trans.transform.translation.x, current_cam_to_base_trans.transform.translation.y,
                                            current_cam_to_base_trans.transform.translation.z])
                self._prev_look_vec = quaternion.as_rotation_matrix(quaternion.quaternion(current_cam_to_base_trans.transform.rotation.w, current_cam_to_base_trans.transform.rotation.x,
                                                            current_cam_to_base_trans.transform.rotation.y, current_cam_to_base_trans.transform.rotation.z))[:,2]

                head_trans = self._tfBuffer.lookup_transform(self._ROBOT_BASE, head_name, rospy.Time())
                # head_cam_trans = self._tfBuffer.lookup_transform('pixel6pro', head_name, rospy.Time())
                head_pos = np.array([head_trans.transform.translation.x, head_trans.transform.translation.y, head_trans.transform.translation.z])
                hip_trans = self._tfBuffer.lookup_transform(self._ROBOT_BASE, spine_name[0], rospy.Time())
                hip_pos = np.array([hip_trans.transform.translation.x, hip_trans.transform.translation.y, hip_trans.transform.translation.z])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # print(f'Cannot find the transform between pixel6pro and pandda_link8')
                self._lookup_rate.sleep()
                continue

            EE_to_cam_transform = self._EE_to_cam_transform
            hands_pos = [np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]) for trans in hands_trans]
            wrists_pos = [np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]) for trans in wrists_trans]

            self._hand_var_est.add_new_data(hands_pos)
            #self._since_last_est += 1
            #if self._since_last_est > self._est_cycle:
            hand_mean, self._hand_var = self._hand_var_est.estimate()
            self._since_last_plan += 1

            ## Framing and angle reset
            
            shoulders_pos = [np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]) for trans in shoulders_trans]
            shoulder_l2r = self._prev_shoulder_dir_filter.update(number_works.normalize_vec(shoulders_pos[1] - shoulders_pos[0]))
            # if (self._dist_mode == 'close' or self._angle_mode == 'top') and len(wrists_pos) > 1 and len(shoulders_trans) > 1:
            #     left_relaxed, right_relaxed = PandaFollowHand.__check_hand_relaxed(hands_pos[0], hands_pos[1], shoulders_pos[0], shoulders_pos[1])
            #     self._left_hand_relaxed.update(left_relaxed)
            #     self._right_hand_relaxed.update(right_relaxed)
            #     if self._left_hand_relaxed.get_val() or self._right_hand_relaxed.get_val():

            if (self._dist_mode == 'close' or self._angle_mode == 'top'):
                left_behind_back, right_behind_back = PandaFollowHand.__check_hand_behind(hands_pos[0], hands_pos[1], hip_pos)
                if left_behind_back or right_behind_back:
                    if self._dist_mode == 'close':
                        self._dist_mode = 'medium'
                        self.__send_cam_zoom(1.0)
                        rospy.loginfo("Reset to medium")
                    if self._angle_mode == 'top':
                        self._angle_mode = 'level'
                        rospy.loginfo("Reset to level")

            if self._since_last_plan > self._plan_cycle:
                ## Only plan path every 200ms
                now = rospy.Time.now().to_sec()

                pointing = self._pointing_detected
                # pointing = 'Both' if (self._prop_pointing_left and self._prop_pointing_right) else ('Left' if self._prop_pointing_left else ('Right' if self._prop_pointing_right else self._pointing_detected))

                facing = self._looking_into_eyes

                robot_state = 'head' if facing else ('orbit' if self._is_orbiting else ('object' if pointing == 'Left' or pointing == 'Right' else 'hand'))
                self.__send_cam_status_disp(pointing=pointing, robot_state=robot_state)

                if self._vary_orientation and self._dist_mode == 'medium':
                    neutral_orientation = number_works.normalize_vec(np.cross(shoulder_l2r, np.array([0, 0, 1])))
                    desired_look_dir = neutral_orientation if self._orientation_ind == 0 else self._other_orientation[max(0, self._orientation_ind)]
                    vary = random.random() < self._vary_orientation_prob
                    if vary:
                        if self._orientation_ind == 0:
                            self._orientation_ind = 1 if random.random() > 0.5 else -1
                            desired_look_dir = self._other_orientation[max(0, self._orientation_ind)]
                        else:
                            self._orientation_ind = 0
                            desired_look_dir = neutral_orientation
                else:
                    desired_look_dir = number_works.normalize_vec(np.cross(shoulder_l2r, np.array([0, 0, 1])))

                desired_pitch = np.pi / 2 if self._angle_mode == 'top' else -1 ## Do not enforce pitch unless it's overhead shot, with the exception of a face shot (enforce 0 in the optimization)
                smooth_co = 2.0 if self._dist_mode == "medium" else 6.0 ## Make it move really slowly in close-up mode
                orientation_co = 0 if self._angle_mode == 'top' else 0.5
                zoom_co = 0 if desired_pitch > 0 else 0.2 # Do not enfore zoom in overhead mode, except in face mode
                pitch_co = 0 if desired_pitch < 0 else 1.0

                # if self._anim_duration['cycle'] > 0:
                #     if pointing != '' or self._angle_mode == 'top' or self._dist_mode == 'close' or facing:
                #         self._angle_variation_func = None
                #     else:
                #         if self._angle_variation_func is None:
                #             ## Restart the function from the closest looking direction
                #             self._angle_variation_func = get_orbit_directions_func(half_rotate_by=np.pi / 4, start_from=number_works.project_vec_to_plane(self._prev_look_vec, np.array([0, 0, 1]), normalize=True))
                #             self._start_ts = now
                #         desired_look_dir = self._angle_variation_func((now -  self._start_ts) / self._anim_duration['cycle'])
                #         orientation_co = 1.0

                #rospy.loginfo("Optimzation starts")
                look_at_point, look_at_dir = self.compute_cost_look_at_hand_op(hands_pos=hands_pos, hands_mean=hand_mean, wrists_pos=wrists_pos, hand_var=self._hand_var, head_pos=head_pos, pointing=pointing, facing=facing,
                                                                            prev_cam=self._prev_cam, desired_dir=desired_look_dir, desired_pitch = desired_pitch,
                                                                            disp_smooth_co=smooth_co, ori_co=orientation_co, zoom_co=zoom_co, pitch_co=pitch_co)
                # look_at_point, look_at_dir = self.compute_cost_look_at_hand_joints(hands_pos=hands_pos, wrists_pos=wrists_pos, hand_var=self._hand_var, head_pos=head_pos, pointing=pointing, facing=facing,
                #                                                             EE_to_cam_trans=EE_to_cam_transform, robot=self._arm,
                #                                                             prev_cam=self._prev_cam, desired_dir=desired_look_dir, desired_pitch = desired_pitch,
                #                                                             disp_smooth_co=smooth_co, ori_co=orientation_co, zoom_co=zoom_co, pitch_co=pitch_co, lookat_co=lookat_co)
                #rospy.loginfo("Optimzation ends")

                self._since_last_plan = 0
                # compute a look position
                cam_x_axis = np.cross(np.array([0, 0, -1]), look_at_dir)
                cam_x_axis = cam_x_axis / linalg.norm(cam_x_axis)
                cam_y_axis = np.cross(look_at_dir, cam_x_axis)
                cam_y_axis = cam_y_axis / linalg.norm(cam_y_axis)
                cam_rot_mat = np.reshape(np.concatenate((cam_x_axis, cam_y_axis, look_at_dir)), (3, -1), 'F') 
                # cam_goal_mat = np.vstack((np.hstack((cam_rot_mat, np.reshape(look_at_pos, (3, 1)))), np.array([0, 0, 0, 1])))
                cam_to_base_mat = self.mat_from_RT(cam_rot_mat, look_at_point)

                EE_to_cam_rot_mat = quaternion.as_rotation_matrix(quaternion.quaternion(EE_to_cam_transform.transform.rotation.w, EE_to_cam_transform.transform.rotation.x,
                    EE_to_cam_transform.transform.rotation.y, EE_to_cam_transform.transform.rotation.z)) ## 3x3 without translation
                EE_to_cam_mat = self.mat_from_RT(EE_to_cam_rot_mat, 
                                                np.array([EE_to_cam_transform.transform.translation.x, EE_to_cam_transform.transform.translation.y, EE_to_cam_transform.transform.translation.z]))

                EE_mat = cam_to_base_mat @ EE_to_cam_mat  # world-to-EE
                EE_rot_q = quaternion.from_rotation_matrix(EE_mat[:3, :3], nonorthogonal=False)
                EE_tranlation = tf_conversions.transformations.translation_from_matrix(EE_mat)

                goto_msg = geometry_msgs.msg.PoseStamped()
                goto_msg.header.frame_id = 'panda_link0'
                goto_msg.header.stamp = rospy.Time.now()
                goto_msg.pose.position.x = EE_tranlation[0]
                goto_msg.pose.position.y = EE_tranlation[1]
                goto_msg.pose.position.z = EE_tranlation[2]
                goto_msg.pose.orientation.x = EE_rot_q.x
                goto_msg.pose.orientation.y = EE_rot_q.y
                goto_msg.pose.orientation.z = EE_rot_q.z
                goto_msg.pose.orientation.w = EE_rot_q.w

                self._action_pub.publish(goto_msg)
            self._lookup_rate.sleep()

def main():
    try:
        follow = PandaFollowHand()
        follow.listen(hands_name=[('person_left_hand', 'person_left_wrist'), ('person_right_hand', 'person_right_wrist')], head_name = 'person_head',
                      shoulders_name=['person_left_shoulder', 'person_right_shoulder'], spine_name=['person_spine_base', 'person_spine_mid'])
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()


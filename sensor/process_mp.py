'''
CV stuffs using MediaPipe, publishing detected event using UDP
'''
from asyncore import loop
import os
from typing import Dict, List, Tuple

from kinect_udp_forward import UdpForward
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2 as cv
import mediapipe as mp
from hand_model import KeyPointClassifier
import numpy as np
import copy
import itertools
import json


class PipeReader(object):
    def __init__(self, cam_num: List[int], width: int, height: int) -> None:
        self._cap0 = cv.VideoCapture(cam_num[0])
        self._cap0.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self._cap0.set(cv.CAP_PROP_FRAME_HEIGHT, height)

        # self._cap1 = cv.VideoCapture(cam_num[1])
        # self._cap1.set(cv.CAP_PROP_FRAME_WIDTH, width)
        # self._cap1.set(cv.CAP_PROP_FRAME_HEIGHT, height)

        # self._caps = [self._cap0, self._cap1]
        self._caps = [self._cap0]
        
        self._mp_hands_0 = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence = 0.7,
            min_tracking_confidence = 0.5
        )

        # self._mp_hands_1 = mp.solutions.hands.Hands(
        #     static_image_mode=False,
        #     max_num_hands=2,
        #     min_detection_confidence = 0.5,
        #     min_tracking_confidence = 0.5
        # )

        # self._mp_hands = [self._mp_hands_0, self._mp_hands_1]
        self._mp_hands = [self._mp_hands_0]

        self._classifier = KeyPointClassifier(model_path='hand_model/keypoint_classifier/keypoint_classifier_custom.tflite')

        # self._mp_face = mp.solutions.face_mesh.FaceMesh(max_num_faces=1,
        #                                                 refine_landmarks=False,
        #                                                 min_detection_confidence=0.7,
        #                                                 min_tracking_confidence=0.7)
        

    @staticmethod
    def calc_bounding_rect(image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_array = np.empty((0, 2), int)

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point = [np.array((landmark_x, landmark_y))]

            landmark_array = np.append(landmark_array, landmark_point, axis=0)

        x, y, w, h = cv.boundingRect(landmark_array)

        return [x, y, x + w, y + h]

    @staticmethod
    def calc_landmark_list(image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        # Keypoint
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            # landmark_z = landmark.z

            landmark_point.append([landmark_x, landmark_y])

        return landmark_point

    @staticmethod
    def pre_process_landmark(landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        # Convert to a one-dimensional list
        temp_landmark_list = list(
            itertools.chain.from_iterable(temp_landmark_list))

        # Normalization
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))

        return temp_landmark_list

    Point = Dict[str, float]
    @staticmethod
    def compute_face_orientation(face_landmarks: List[Point]) -> List[float]:
        '''
        Following https://github.com/tensorflow/tfjs-models/pull/844
        '''
        p1, p2, p3 = face_landmarks[127], face_landmarks[356], face_landmarks[6]
        vx = np.array([p2['x'] - p1['x'], p2['y'] - p1['y'], p2['z'] - p1['z']])
        vhelp = np.array([p3['x'] - p1['x'], p3['y'] - p1['y'], p3['z'] - p1['z']])
        vy = np.cross(vhelp, vx)
        vz = np.cross(vx, vy)
        # vx = vx / np.linalg.norm(vx)
        # vy = vy / np.linalg.norm(vy)
        vz = vz / np.linalg.norm(vz)

        ## Flip around camera y axis
        vz[0] = -vz[0]
        #print(f"Forward vector of the face is {vz}")

        return vz.tolist()

    def ReadOnce(self, cap_ind) -> Tuple[List[str], bool, List[float]]:
        '''
        Read one frame and perform detection
        Return whether a pointing gesture is found and which hand it is
        '''

        
        ret, image = self._caps[cap_ind].read()
        if not ret:
            return [], False, []

        handedness_res = []
        face_found = False
        face_orientation = []

        image = cv.flip(image, 1)  # Mirror display
        # debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        hand_results = self._mp_hands[cap_ind].process(image)
        face_results = self._mp_face.process(image)
        image.flags.writeable = True

        if hand_results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(hand_results.multi_hand_landmarks,
                                                  hand_results.multi_handedness):
                # Bounding box calculation
                # brect = PipeReader.calc_bounding_rect(debug_image, hand_landmarks)
                # Landmark calculation
                # landmark_list = PipeReader.calc_landmark_list(debug_image, hand_landmarks)
                landmark_list = PipeReader.calc_landmark_list(image, hand_landmarks)

                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = PipeReader.pre_process_landmark(
                    landmark_list)

                hand_sign_id, confidence = self._classifier(pre_processed_landmark_list)

                if hand_sign_id == 1 and confidence > 0.88:
                #    continue
                #else:
                    handedness_res.append(handedness.classification[0].label)
                    print(f"Cam {cap_ind} detecting {handedness.classification[0].label} with confidence {confidence}")
                #    break
        
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                face_found = True
                all_landmarks = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in face_landmarks.landmark]
                #print(f"Landmark 0 is {all_landmarks[0]} and size {len(all_landmarks)}")
                face_orientation = PipeReader.compute_face_orientation(all_landmarks)

        return handedness_res, face_found, face_orientation

    def ReadHandOnce(self, cap_ind) -> List[str]:
        ret, image = self._caps[cap_ind].read()
        if not ret:
            return []

        handedness_res = []
        face_found = False
        face_orientation = []

        image = cv.flip(image, 1)  # Mirror display
        # debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        hand_results = self._mp_hands[cap_ind].process(image)
        #face_results = self._mp_face.process(image)
        image.flags.writeable = True

        if hand_results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(hand_results.multi_hand_landmarks,
                                                  hand_results.multi_handedness):
                # Bounding box calculation
                # brect = PipeReader.calc_bounding_rect(debug_image, hand_landmarks)
                # Landmark calculation
                # landmark_list = PipeReader.calc_landmark_list(debug_image, hand_landmarks)
                landmark_list = PipeReader.calc_landmark_list(image, hand_landmarks)

                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = PipeReader.pre_process_landmark(
                    landmark_list)

                hand_sign_id, confidence = self._classifier(pre_processed_landmark_list)

                if hand_sign_id == 1 and confidence > 0.85:
                #    continue
                #else:
                    handedness_res.append(handedness.classification[0].label)
                    print(f"Cam {cap_ind} detecting {handedness.classification[0].label} with confidence {confidence}")
                #    break
        return handedness_res

    # def ReadFaceOnce(self, cap_ind:int) -> Tuple[bool, List[float]]:
    #     ret, image = self._caps[cap_ind].read()
    #     if not ret:
    #         return False, []

    #     #handedness_res = []
    #     face_found = False
    #     face_orientation = []

    #     image = cv.flip(image, 1)  # Mirror display
    #     # debug_image = copy.deepcopy(image)

    #     image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    #     image.flags.writeable = False
    #     #hand_results = self._mp_hands[cap_ind].process(image)
    #     face_results = self._mp_face.process(image)
    #     image.flags.writeable = True

    #     if face_results.multi_face_landmarks:
    #         for face_landmarks in face_results.multi_face_landmarks:
    #             face_found = True
    #             all_landmarks = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in face_landmarks.landmark]
    #             #print(f"Landmark 0 is {all_landmarks[0]} and size {len(all_landmarks)}")
    #             face_orientation = PipeReader.compute_face_orientation(all_landmarks)

    #     return face_found, face_orientation

if __name__ == '__main__':
    looping = True
    udp_forward = UdpForward('your.ip.here', 12346)
    pipe_reader = PipeReader(cam_num=[0], width=960, height=720)

    def make_msg(handedness: List[str], face_found: bool, face_ori: List[float]):
        return json.dumps({
            'handedness': handedness,
            'face_found': face_found,
            'face_orientation': face_ori
        })

    while looping:
        try:
            handedness_0 = pipe_reader.ReadHandOnce(0)
            # face_found_1, face_ori_1 = pipe_reader.ReadFaceOnce(1)
            #handedness_1, face_found_1, face_ori_1 = pipe_reader.ReadOnce(1)
            #handedness, face_found, face_ori = handedness_0 if handedness_0 == handedness_1 else [], face_found_0, face_ori_0
            #print(f"Detection results: {handedness} \n face found {face_found} facing {face_ori}")
            print(f"Pointing fingers are {handedness_0}")
            # print(f"Face are {face_ori_1}")
            udp_forward.forward_str(make_msg(handedness_0, face_found=False, face_ori=[]))
        except KeyboardInterrupt:
            looping = False
            print("Exiting because of keyboard interrupt ...")



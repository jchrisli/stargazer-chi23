#!/usr/bin/env python3.8

'''
A node that tracks hands (use AruCo markers for now)
Publish hand positions in the robot base frame
Subsrcibe to the camera stream (camera/raw_image)
'''
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import numpy as np
import tf_conversions
import yaml
import sys

class HandTracker(object):
    def __init__(self, tag_ids, marker_phys_size, cam_mat, cam_dist, parent_frame) -> None:
        self._tags_to_track = tag_ids
        self._marker_phys_size = marker_phys_size
        self._cam_mat = cam_mat
        self._cam_dist = cam_dist

        # initialize Aruco parameters
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self._aruco_param = aruco.DetectorParameters_create()

        self._node = rospy.init_node("stargazer_hand_tracker")

        self._image_sub = rospy.Subscriber("/camera/c930_cam/image_raw", Image, self.find_hand)
        self._cv_bridge = CvBridge()

        # initialize a tf broadcaster
        self._br = tf2_ros.TransformBroadcaster()
        self._parent_frame = parent_frame
        self._tracked_frame = 'tracked_hand'

    @classmethod
    def __rvec_to_q(cls, rv):
        rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rv)
        q = tf_conversions.transformations.quaternion_from_matrix(rotation_matrix)

        return q

    def __construct_tf_msg(self, trans_v, rot_q):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._parent_frame # parent id
        t.child_frame_id = self._tracked_frame
        # print(f"shape of trans_v is {trans_v.shape}")
        t.transform.translation.x = trans_v[0,0]
        t.transform.translation.y = trans_v[0,1]
        t.transform.translation.z = trans_v[0,2]
        t.transform.rotation.x = rot_q[0]
        t.transform.rotation.y = rot_q[1]
        t.transform.rotation.z = rot_q[2]
        t.transform.rotation.w = rot_q[3]

        return t

    def find_hand(self, image_in):
        # Convert ROS image to OpenCV image
        try:
          cv_image = self._cv_bridge.imgmsg_to_cv2(image_in, "passthrough")
        except CvBridgeError as e:
          rospy.logerr("CvBridge Error: {0}".format(e))

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters = self._aruco_param)
        #print(f"Marker ids are {ids}")
        ids = [ii[0] for ii in ids] if ids is not None else []

        selected_ids = [a for a in ids if a in self._tags_to_track]
        selected = [corners[i] for i in range(0, len(corners)) if ids[i] in self._tags_to_track]
        if len(selected_ids) > 0:
            rv, tv, _o = aruco.estimatePoseSingleMarkers(selected, self._marker_phys_size, self._cam_mat, self._cam_dist)
            rq = [HandTracker.__rvec_to_q(rvec) for rvec in rv] # convert to rotation matrix

            res = zip(selected_ids, rq, tv) # (tagid, rotation_mat, translation_vec)
            #print(list(res))
            #print(ids)
            for tracked in res:
                tracked_msg = self.__construct_tf_msg(tracked[2], tracked[1]) # TODO: pass id
                self._br.sendTransform(tracked_msg)

if __name__ == '__main__':
    # TODO: read this from the camera_info topic, maybe a service?
    file_path = '/home/jiannanli/Factory/armros/src/stargazer/stargazer_watcher/data/c930-720p-caminfo.yaml'
    with open(file_path) as fr:
        c = yaml.load(fr)
        cam_mat = np.array(c['camera_matrix'])
        cam_dist = np.array(c['dist_coefs'])


    tags = [24]
    phys_size = 0.04 # 40mm
    parent_frame_name = 'c930_camera'
    tracker = HandTracker(tags, phys_size, cam_mat, cam_dist, parent_frame_name)

    rospy.spin()






        




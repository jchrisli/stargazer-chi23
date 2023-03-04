#!/usr/bin/env python3.8
'''
A node that publishes the camera intrinsic parameters from a file
'''

"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
import sys

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    # See http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]

    # flatten matrix into array
    intrinsic_vec = [n for row in calib_data["camera_matrix"] for n in row]
    print(intrinsic_vec)
    camera_info_msg.K = intrinsic_vec
    dc = calib_data["dist_coefs"]
    camera_info_msg.D = dc[0]
    camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    camera_info_msg.P = [intrinsic_vec[0], intrinsic_vec[1], intrinsic_vec[2], 0, 
                        intrinsic_vec[3], intrinsic_vec[4], intrinsic_vec[5], 0,
                        0, 0, 1, 0]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    # import argparse
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument("filename", help="Path to yaml file containing " +\
    #                                          "camera calibration data")
    # args = arg_parser.parse_args()
    filename = sys.argv[1]

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()
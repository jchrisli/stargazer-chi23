#!/usr/bin/env python3

import torch
from PIL import Image, ImageOps
import rospy
import imutils
import matplotlib.pyplot as plt
from teleop_msgs.srv import GetObjectShape, GetObjectShapeResponse, GetObjectShapeRequest
from teleop_msgs.msg import DetectorInference
import numpy as np


class ObjectDetector:
    def __init__(self):
        self.node = rospy.init_node('image_to_shape_server_node')
        self.image_service = rospy.Service('image_to_shape_service', GetObjectShape, self.get_obj_shape)
        self.model_dir = '/home/karthikm/robot-perception/robot-perception-project/model/best.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_dir)
        # Load model for inference
        self.IMG_SIZE = 640
        print("Image to color service now launched")
        rospy.spin()

    def get_obj_shape(self, req):
        print("Received img request")
        img_height = req.image.width
        img_width = req.image.height
        image = Image.frombytes('RGBA', (img_width, img_height), req.image.data)
        image = self.convert_from_image_to_cv2(image)
        resized_image = imutils.resize(image, width=640)
        print(resized_image.shape)
        results = self.model(resized_image, size=640)

        response = GetObjectShapeResponse()
        response.inferences = []
        #size = len(results.xyxy[0])
        #response.inferences = DetectorInference[size]

        for result in results.xyxy[0]:
            # Get the normalized x, y, w, h
            x_min = result[0].item()
            y_min = result[1].item()
            x_max = result[2].item()
            y_max = result[3].item()

            x = x_min
            y = y_min
            w = x_max - x_min
            h = y_max - y_min

            x = x_min / resized_image.shape[1]
            y = y_min / resized_image.shape[0]
            w = w / resized_image.shape[1]
            h = h / resized_image.shape[0]

            inference = DetectorInference()
            inference.x.data = x
            inference.y.data = y
            inference.w.data = w
            inference.h.data = h
            inference.confidence.data = result[4].item()
            inference.obj_class.data = result[5].item()
            response.inferences.append(inference)
            print(inference.obj_class.data)
            print(inference.confidence.data)

        return response

    def convert_from_image_to_cv2(self, img: Image) -> np.ndarray:
        return np.asarray(img)


def main():
    ObjectDetector()


if __name__ == '__main__':
    main()

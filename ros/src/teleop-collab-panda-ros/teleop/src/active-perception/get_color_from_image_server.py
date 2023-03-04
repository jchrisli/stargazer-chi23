#!/usr/bin/env python3

import rospy
from teleop_msgs.srv import GetObjectColor, GetObjectColorResponse
import torch
from PIL import Image, ImageOps
import torchvision
from keras.models import load_model
from tensorflow.keras.preprocessing import image
import numpy as np
import cv2
from tensorflow import keras


class ShapeFromPoses:
    def __init__(self):
        self.node = rospy.init_node('image_to_color_server_node')
        self.image_service = rospy.Service('image_to_color_service', GetObjectColor, self.get_obj_shape)
        print("Image to color service now launched")

        # Load model for inference
        self.dir = "/home/karthikm/teleop_collab_ws/src/teleop-collab-panda-ros/teleop/src/active-perception/model/"
        self.model = load_model("/home/karthikm/teleop_collab_ws/src/teleop-collab-panda-ros/teleop/src"
                                "/active-perception/model/shape_model.h5")
        self.IMG_SIZE = 224

        rospy.spin()

    def prepare_image(self):
        img_path = ''
        img = image.load_img(self.image_path, target_size=(self.IMG_SIZE, self.IMG_SIZE))
        img_array = image.img_to_array(img)
        img_array_expanded_dims = np.expand_dims(img_array, axis=0)
        return keras.applications.mobilenet.preprocess_input(img_array_expanded_dims)

    def get_obj_shape(self, req):

        # Save image
        self.save_image(req)

        # Run model
        preprocessed_image = self.prepare_image()
        predictions_shape = self.model.predict(preprocessed_image)
        labels = ['Cube', 'Cylinder', 'Spheroid', 'Sphere']
        shape = labels[predictions_shape[0].tolist().index(max(predictions_shape[0]))]
        print("Shape: ", shape)
        res = GetObjectColorResponse()
        res.color.data = shape
        return res

    def save_image(self, req):
        image_height = req.image.width
        image_width = req.image.height
        image = Image.frombytes('RGBA', (image_width, image_height), req.image.data)
        image = ImageOps.flip(image)
        self.image_name = "Input.png"
        self.image_path = self.dir + self.image_name
        image.save(self.image_path)
        print("Saved image")


    '''def get_transform(self):
        transform = torchvision.transforms.Compose(
            [
                torchvision.transforms.Resize(
                    (
                        224,
                        224,
                    )
                ),
                torchvision.transforms.ToTensor(),
            ]
        )
        return transform

    def pre_process_image(self):
        image_origin = Image.open(self.image_path).convert("RGB")
        transform = self.get_transform()
        image = [transform(image_origin).unsqueeze(0)]
        image = list(img.to(self.device) for img in image)
        return image

    def run_model(self):
        image = self.pre_process_image()
        self.model(torch.stack(image).reshape(-1, 3, 224, 224))'''


def main():
    ShapeFromPoses()


if __name__ == '__main__':
    main()

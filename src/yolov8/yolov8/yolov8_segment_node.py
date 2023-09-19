#
# Copyright 2023 Ettore Sani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import cv2
import torch
import random
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data

from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool


class Yolov8SegmentNode(Node):
	# This node implements YOLOv8 image segmentation in ROS 2 Humble.
	# It subscribes to an image, depth image and camera_info topics, checking their synchronization with the message_filters package.
	# In the callback, it does the inference on the image, detecting the traversable area in the image.
	# It publishes a segmented depth image only of the traversable zone.
	# If the debug parameters is set to true, it also prints on screen the result of the segmentation.

    def __init__(self) -> None:
        super().__init__("yolov8_segment_node")

		# Node parameters
        self.declare_parameter("model_path", "/home/ucmrobotics/AgBot/src/husky_vision/yolov8/models/huskynav_yolov8_1024x576.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("frequency", 2.0)
        self.declare_parameter("queue_size", 30)
        self.declare_parameter("delay_threshold", 0.3)
        self.declare_parameter("NN_threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("debug", True)

        model_path        = self.get_parameter("model_path").get_parameter_value().string_value
        device            = self.get_parameter("device").get_parameter_value().string_value
        frequency         = self.get_parameter("frequency").get_parameter_value().double_value
        queue_size        = self.get_parameter("queue_size").get_parameter_value().integer_value
        delay_threshold   = self.get_parameter("delay_threshold").get_parameter_value().double_value
        self.NN_threshold = self.get_parameter("NN_threshold").get_parameter_value().double_value
        self.enable       = self.get_parameter("enable").get_parameter_value().bool_value
        self.debug        = self.get_parameter("debug").get_parameter_value().bool_value

        self.cv_bridge = CvBridge()     # Initialize the OpenCV2 bridge
        self.model = YOLO(model_path)   # Initialize the YOLOv8 model
        self.model.to(device)           # If the inference should be done on cpu or gpu

        # Publishers and Subscribers
        self._res_pub   = self.create_publisher(Image, "/image_seg/depth", 10)
        self._info_pub  = self.create_publisher(CameraInfo, "/image_seg/info", 10)
        self._color_sub = Subscriber(self, Image, "/color/image")
        self._depth_sub = Subscriber(self, Image, "/stereo/depth")
        self._info_sub  = Subscriber(self, CameraInfo, "/stereo/camera_info")

		# From ROS 2 message_filters, to synchronize the topics
        self._time_sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub, self._info_sub],   # Synchronized subscribers
            queue_size,
            delay_threshold,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self._time_sync.registerCallback(self.image_cb)

        # Services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

		# Create a timer to mantain the desired frequency
        self._timer_expired = True
        self._timer = self.create_timer(1/frequency, self.timer_callback)

        self.get_logger().info("yolov8 node initialized")

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
		# To enable/disable the node
        self.enable = req.data
        res.success = True
        return res

    def timer_callback(self) -> None:
		# To mantain the desired frequncy of the inference
        self._timer_expired = True

    def image_cb(self, color: Image, depth: Image, info: CameraInfo) -> None:

        if self.enable and self._timer_expired:
            self._timer_expired = False
            # Convert images in OpenCV2 format
            color_img = self.cv_bridge.imgmsg_to_cv2(color)
            depth_img = self.cv_bridge.imgmsg_to_cv2(depth)

            # Do the inference on the model with YOLOv8 on the RGB image
            results = self.model.predict(source=color_img, conf=self.NN_threshold, imgsz=(1024,576), rect=True, verbose=True)

			# If there are no detections, return
            if len(results[0]) == 0:
                return

            mask = np.zeros((576, 1024), dtype="uint8")   # Initialize mask

            for i, mask_i in enumerate(results[0].masks):
                if results[0].boxes[i].cls == 1:                    # If the result is an object of class traversable
                    mask_i_np = mask_i.cpu().data.numpy().transpose(1,2,0).astype(np.uint8)   # Mask of inferred object
                    mask_i_np = cv2.resize(mask_i_np, (1024,576))   # For safety, resize the mask to the same size as the image
                    mask = cv2.bitwise_or(mask, mask_i_np)          # Overlay all traversable masks, to deal with the case where more than one traversable objects are detected

            depth_msk = cv2.bitwise_and(depth_img, depth_img, mask=mask)   # Create a mask of the depth image of the detected traversable area

			# Create and publish a message with the new masked depth image
            depth_msk_msg = self.cv_bridge.cv2_to_imgmsg(depth_msk)
            depth_msk_msg.header = depth.header
            self._res_pub.publish(depth_msk_msg)
            self._info_pub.publish(info)   # Publish the camera info as the original one

            if self.debug == True:   # Print debug images
                green = np.array([0,255,0], dtype='uint8')
                red   = np.array([0,0,255], dtype='uint8')
                mask_colored = np.zeros((576, 1024, 3), dtype="uint8")     # Initialize the colored mask for all the image
                annotator = Annotator(color_img, line_width=0, pil=True)   # Use YOLOv8 annotator for customizing image output
                for i, mask_i in enumerate(results[0].masks):
                    box_i = results[0].boxes[i]
                    mask_i_np = mask_i.cpu().data.numpy().transpose(1,2,0).astype(np.uint8)
                    mask_i_np = cv2.resize(mask_i_np, (1024,576))
                    if box_i.cls == 0:     # If the detection is an obstacle
                        mask_colored = np.where(mask_i_np[...,None], red, mask_colored)    # Overlay a red mask
                        label = f'{"obstacle"} {float(box_i.conf):.2f}'
                        annotator.box_label(box_i.xyxy[0], label=label, color=(0, 0, 255), txt_color=(255, 255, 255))   # Annotate the red bounding box with the confidence
                    elif box_i.cls == 1:   # If the detection is a traversable area
                        mask_colored = np.where(mask_i_np[...,None], green, mask_colored)   # Overlay a green mask
                        label = f'{"traversable"} {float(box_i.conf):.2f}'
                        annotator.box_label(box_i.xyxy[0], label=label, color=(0, 255, 0), txt_color=(255, 255, 255))   # Annotate the green bounding box with the confidence

                color_msk = cv2.addWeighted(annotator.result(), 0.8, mask_colored, 0.3, 0)   # Merge the coloured mask with the original image
                cv2.imshow("yolov8 results", color_msk)   # Display the image masked with colors, bounding boxes, and annotated precisions
                cv2.imshow("depth masked", depth_msk)     # Display the depth image masked
                cv2.waitKey(1)

def main():
    try:
        rclpy.init()
        node = Yolov8SegmentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node Yolov8SegmentNode shutdown')
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

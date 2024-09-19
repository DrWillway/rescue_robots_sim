#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import torch
from pathlib import Path

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from yolobot_recognition.models.common import DetectMultiBackend
from yolobot_recognition.utils.general import check_img_size, non_max_suppression, scale_boxes
from yolobot_recognition.utils.plots import Annotator, colors
from yolobot_recognition.utils.torch_utils import select_device


class YOLOv5Node(Node):

    def __init__(self):
        super().__init__('yolov5_node')

        # Declare and read parameters
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('weights', 'yolov5s.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('device_num', '')
        self.declare_parameter('imgsz', [640, 480])

        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        weights = self.get_parameter('weights').get_parameter_value().string_value
        conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        iou_thres = self.get_parameter('iou_thres').get_parameter_value().double_value
        device_num = self.get_parameter('device_num').get_parameter_value().string_value
        imgsz = tuple(self.get_parameter('imgsz').get_parameter_value().integer_array_value)

        self.get_logger().info(f'Yolo namespace: {namespace}')

        # Initialize device and model
        self.device = select_device(device_num)
        self.model = DetectMultiBackend(weights, device=self.device)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        # Warm up model
        self.model.warmup(imgsz=(1, 3, *self.imgsz))  # warmup

        self.detected_persons_odom = set()

        # ROS setup
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            f'/{namespace}/bottom/image_raw',
            self.camera_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            f'/{namespace}/odom',
            self.odom_callback,
            10
        )

        self.human_detected_pose_publisher = self.create_publisher(Point, f'/{namespace}/detected_pose', 10)

        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

        self.get_logger().info("YOLOv5 node has been initialized")

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), scaleup=True, stride=32):
        """Resize and pad image while meeting stride-multiple constraints"""
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, (r, r), (dw, dh)

    def odom_callback(self, msg):
        self.odom = msg
        # self.get_logger().info(f"Received odometry data: {msg}")
        
    def camera_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Letterbox resize
        img0 = img.copy()
        img, _, _ = self.letterbox(img, new_shape=self.imgsz, stride=self.stride)

        # Stack
        img = img[np.newaxis, :, :, :]

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0

        # Inference
        pred = self.model(img, augment=False)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=None, agnostic=False)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            annotator = Annotator(img0, line_width=3, example=str(self.names))
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = f'{self.names[c]} {conf:.2f}'
                    percentage = float(f'{conf:.2f}')
                    detected_label = self.names[c]
                    if detected_label == 'person':
                        print(f'Detected {detected_label}: {percentage}, at {self.odom.pose.pose.position}')
                        # Creating a new Point object
                        person_pose = Point()
                        person_pose.x = self.odom.pose.pose.position.x
                        person_pose.y = self.odom.pose.pose.position.y
                        self.detected_persons_odom.add((person_pose.x, person_pose.y, person_pose.z))
                        if percentage > 0.5:
                            self.human_detected_pose_publisher.publish(person_pose)
                        # print(self.detected_persons_odom)
                        
                        annotator.box_label(xyxy, label, color=colors(c, True))
            cv2.imshow("YOLOv5 Detection", img0)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv5Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3 
import rclpy # enable use of ros 
from rclpy.node import Node # import node class 
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Twist
import json
import time

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

class CameraPublishers(Node):
    def __init__(self):
        super().__init__("camera_publishers")
        self.front_publisher_ = self.create_publisher(Image, '/video/front_camera', 1)
        self.back_publisher_ = self.create_publisher(Image, '/video/back_camera', 1)
        # self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.realsense_callback, 1)
        # self.vel_subscription = self.create_subscription(Twist, '/wheel_velocity', self.wheel_velocity_callback, 1)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.publishers_callback)

        self.front_fisheye = cv2.VideoCapture(7, cv2.CAP_V4L2) # Fisheye Front
        self.back_boxcam = cv2.VideoCapture(5, cv2.CAP_V4L2) # Back Boxcamera

        self.front_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.front_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.back_boxcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.back_boxcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.bridge = CvBridge()

    def __del__(self):
        try:
            self.front_fisheye.release()
            # self.back_boxcam.release()
        except Exception as e:
            print("Releasing video devices exception: ", e)

    def publishers_callback(self):
        # tic = time.perf_counter()

        try:
            front_frame = Image()
            back_frame = Image()

            _, front_image = self.front_fisheye.read()
            _, back_image = self.back_boxcam.read()

            #print("Front Image", front_image.shape)
            # print("Back Image", back_image.shape)

            front_frame = self.bridge.cv2_to_imgmsg(front_image, 'bgr8')
            back_frame = self.bridge.cv2_to_imgmsg(back_image, 'bgr8')

            self.front_publisher_.publish(front_frame)
            self.back_publisher_.publish(back_frame)


        except Exception as e:
            self.get_logger().info('Exception: "%s"' % e)

        # toc = time.perf_counter()
        # print("Time to process: ", toc-tic)
            

def main():
    rclpy.init(args=None)    # strts ros2 communication, it is mandatory 
    camera_publishers_node = CameraPublishers()
    rclpy.spin(camera_publishers_node)        # pauses program, keeps node running, so no shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
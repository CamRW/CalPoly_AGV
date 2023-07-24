#!/usr/bin/env python3 
import rclpy # enable use of ros 
from rclpy.node import Node # impor node class 
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import json

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

camera_dict = dict()

with open('/workspaces/isaac_ros-dev/configs/camera_configs/cameras.json') as camera_json:
    camera_dict = json.load(camera_json)

class ImageStitching(Node):
    
    def __init__(self):
        super().__init__("image_stitching")
        self.publisher_ = self.create_publisher(Image, '/video/stitched_frames', 1)
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.realsense_callback, 1)
        timer_period = 0.03 #seconds
        self.timer = self.create_timer(timer_period, self.stitching_callback)

        self.cap1 = cv2.VideoCapture(camera_dict['/dev/back_fisheye'], cv2.CAP_V4L2) # Fisheye Back
        self.cap2 = cv2.VideoCapture(camera_dict['/dev/left_fisheye'], cv2.CAP_V4L2) # Left, video6
        self.cap3 = cv2.VideoCapture(camera_dict['/dev/right_fisheye'], cv2.CAP_V4L2) # Right, video12


        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.cap3.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.bridge = CvBridge()
        self.realsense_image = np.empty([480, 640, 3]).astype('uint8')
        self.stitched_image = np.empty([960, 1920, 3]).astype('uint8') #np.empty([1920, 960])

    def __del__(self):
        try:
            self.cap1.release()
            self.cap2.release()
            self.cap3.release()
        except Exception as e:
            print("Releasing video devices exception: ", e)


    def realsense_callback(self, msg):
        self.realsense_image = self.bridge.imgmsg_to_cv2(msg)
        self.realsense_image = cv2.cvtColor(self.realsense_image, cv2.COLOR_RGB2BGR)
        
    def stitching_callback(self):

        try:
            stitched_frames = Image()
            _, frame1 = self.cap1.read()
            _, frame2 = self.cap2.read()
            _, frame3 = self.cap3.read()
        
            frame1 = np.pad(frame1, pad_width=((0,0),(640,640),(0,0)))
            merged_frames = np.concatenate((frame2, self.realsense_image, frame3), axis=1)
            merged_frames = np.concatenate((merged_frames, frame1), axis=0)
            merged_frames = cv2.resize(merged_frames, dsize=(720,480), interpolation=cv2.INTER_LINEAR)
            stitched_frames = self.bridge.cv2_to_imgmsg(merged_frames, 'bgr8')

            self.publisher_.publish(stitched_frames)
        except Exception as e:
            print(e)

    
def main(args=None):
    rclpy.init(args=args)    # strts ros2 communication, it is mandatory 
    image_stitching_node = ImageStitching()
    rclpy.spin(image_stitching_node)        # pauses program, keeps node running, so no shutdown
    rclpy.shutdown()

if __name__=="__main__":
    main()

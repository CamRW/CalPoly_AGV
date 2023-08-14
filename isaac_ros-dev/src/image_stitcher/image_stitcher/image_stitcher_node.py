#!/usr/bin/env python3 
import rclpy # enable use of ros 
from rclpy.node import Node # import node class 
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import json

PI = 3.1415

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

camera_dict = dict()

with open('/workspaces/isaac_ros-dev/configs/camera_configs/cameras.json') as camera_json:
    camera_dict = json.load(camera_json)

class ImageStitching(Node):
    
    def __init__(self):
        super().__init__("image_stitching")
        self.stitched_publisher_ = self.create_publisher(Image, '/video/stitched_frames', 1)
        # self.front_publisher_ = self.create_publisher(Image, '/video/front_camera', 1)
        self.left_publisher_ = self.create_publisher(Image, '/video/left_camera', 1)
        self.right_publisher_ = self.create_publisher(Image, '/video/right_camera', 1)
        # self.back_publisher_ = self.create_publisher(Image, '/video/back_camera', 1)
        # self.rs_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.realsense_callback, 1)
        self.front_subscription = self.create_subscription(Image, '/video/front_camera', self.front_callback, 1)
        self.back_subscription = self.create_subscription(Image, '/video/back_camera', self.back_callback, 1)
        # self.vel_subscription = self.create_subscription(Twist, '/wheel_velocity', self.wheel_velocity_callback, 1)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.stitching_callback)

        self.vel = 0.0
        self.steering_angle = 0.0

        # self.front_fisheye = cv2.VideoCapture(camera_dict['/dev/front_fisheye'], cv2.CAP_V4L2) # Fisheye Front
        self.left_fisheye = cv2.VideoCapture(camera_dict['/dev/left_fisheye'], cv2.CAP_V4L2) # Left
        self.right_fisheye = cv2.VideoCapture(camera_dict['/dev/right_fisheye'], cv2.CAP_V4L2) # Right
        # self.back_boxcam = cv2.VideoCapture(camera_dict['/dev/back_boxcam'], cv2.CAP_V4L2) # Back Boxcamera


        # self.front_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        # self.front_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.left_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.left_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.right_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.right_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        # self.back_boxcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        # self.back_boxcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.bridge = CvBridge()
        # self.realsense_image = np.empty([480, 640, 3]).astype('uint8')
        self.front_image = np.empty([240, 320, 3]).astype('uint8')
        # self.left_image = np.empty([480, 640, 3]).astype('uint8')
        # self.right_image = np.empty([480, 640, 3]).astype('uint8')
        self.back_image = np.empty([240, 320, 3]).astype('uint8')
        # self.realsense_image = np.pad(self.realsense_image, pad_width=((0,0),(640,640),(0,0)))
        self.stitched_image = np.empty([960, 1920, 3]).astype('uint8') #np.empty([1920, 960])

    def __del__(self):
        try:
            # self.front_fisheye.release()
            self.left_fisheye.release()
            self.right_fisheye.release()
            # self.back_boxcam.release()
        except Exception as e:
            print("Releasing video devices exception: ", e)

    # def wheel_velocity_callback(self, msg):
    #     self.vel = round((msg.linear.y/60)*(.3*PI),2) # Converstion from RPM to m/s, wheel diameter is .3 meters
    #     self.steering_angle = round(((msg.angular.z*360)/1023 - 180),2)


    # def realsense_callback(self, msg):
    #     self.realsense_image = self.bridge.imgmsg_to_cv2(msg)
    #     self.realsense_image = cv2.cvtColor(self.realsense_image, cv2.COLOR_RGB2BGR)

    def front_callback(self, msg):
        self.front_image = self.bridge.imgmsg_to_cv2(msg)
        #self.front_image = cv2.cvtColor(self.front_image, cv2.COLOR_RGB2BGR)
        self.front_image = cv2.resize(self.front_image, dsize=(640,480), interpolation=cv2.INTER_LINEAR)

    def back_callback(self, msg):
        self.back_image = self.bridge.imgmsg_to_cv2(msg)
        #self.back_image = cv2.cvtColor(self.back_image, cv2.COLOR_RGB2BGR)
        self.back_image = cv2.resize(self.back_image, dsize=(640,480), interpolation=cv2.INTER_LINEAR)

        
    def stitching_callback(self):

        try:
            stitched_frames = Image()
            # front_frame = Image()
            left_frame = Image()
            right_frame = Image()
            # back_frame = Image()

            # _, front_image = self.front_fisheye.read()
            _, left_image = self.left_fisheye.read()
            _, right_image = self.right_fisheye.read()
            # _, back_image = self.back_boxcam.read()
        
            back_image_padded = np.pad(self.back_image, pad_width=((0,0),(640,640),(0,0)))
            merged_frames = np.concatenate((left_image, self.front_image, right_image), axis=1)
            merged_frames = np.concatenate((merged_frames, back_image_padded), axis=0)
            merged_frames = cv2.resize(merged_frames, dsize=(720,480), interpolation=cv2.INTER_LINEAR)
            # merged_frames = cv2.putText(merged_frames, 'Velocity ' + str(self.vel) + " m/s", (25,380), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 0, 0), 2, cv2.LINE_AA)
            # merged_frames = cv2.putText(merged_frames, 'Steering ' + str(self.steering_angle) + ' deg', (25,430), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 0, 0), 2, cv2.LINE_AA)
            stitched_frames = self.bridge.cv2_to_imgmsg(merged_frames, 'bgr8')
            # front_frame = self.bridge.cv2_to_imgmsg(front_image, 'bgr8')
            left_frame = self.bridge.cv2_to_imgmsg(left_image, 'bgr8')
            right_frame = self.bridge.cv2_to_imgmsg(right_image, 'bgr8')
            # back_frame = self.bridge.cv2_to_imgmsg(back_image, 'bgr8')


            self.stitched_publisher_.publish(stitched_frames)
            # self.front_publisher_.publish(front_frame)
            self.left_publisher_.publish(left_frame)
            self.right_publisher_.publish(right_frame)
            # self.back_publisher_.publish(back_frame)

        except Exception as e:
            print(e)

    
def main(args=None):
    rclpy.init(args=args)    # strts ros2 communication, it is mandatory 
    image_stitching_node = ImageStitching()
    rclpy.spin(image_stitching_node)        # pauses program, keeps node running, so no shutdown
    rclpy.shutdown()

if __name__=="__main__":
    main()
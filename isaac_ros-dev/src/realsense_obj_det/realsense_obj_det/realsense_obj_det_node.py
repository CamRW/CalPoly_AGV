import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from std_msgs.msg import Int16
from cv_bridge import CvBridge


class RealsenseObjDet(Node):

    def __init__(self):
        super().__init__('realsense_obj_det')
        self.publisher_ = self.create_publisher(Int16, 'object_det', 10)
        self.subscription = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.listener_callback, 10)

        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.depth_array = np.empty([640, 480])
        #self.int_msg = Int16()
    
    def timer_callback(self):
        msg = Int16()
        depth_idxs = np.argwhere((self.depth_array[100:-100] < 1500) & (self.depth_array[100:-100] > 0))
        if depth_idxs.size > 0:
            if depth_idxs.shape[0] > 2500:
                #print("Object Detected")
                msg.data = 1
            else:
                msg.data = 0
        else:
                msg.data = 0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.depth_array = self.bridge.imgmsg_to_cv2(msg)
        # Int_msg = Int16()
        # depth_idxs = np.argwhere((self.depth_array[100:-100] < 1500) & (self.depth_array[100:-100] > 0))
        # if depth_idxs.size > 0:
        #     if depth_idxs.shape[0] > 2500:
        #         print("Object Detected")
        #         Int_msg.data = 1
        # else:
        #     Int_msg.data = 0
        # self.publisher_.publish(Int_msg)


def main(args=None):
    rclpy.init(args=args)
    realsense_obj_node = RealsenseObjDet()
    rclpy.spin(realsense_obj_node)

    realsense_obj_node.detroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

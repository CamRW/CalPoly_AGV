import torch
import numpy as np
import torch.nn as nn
import cv2
# from torchvision import transforms
import albumentations
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from road_segmentation import unet

transform = albumentations.Compose([
    albumentations.Resize(224, 224, always_apply=True),
    albumentations.Normalize(
            mean=[0.45734706, 0.43338275, 0.40058118],
            std=[0.23965294, 0.23532275, 0.2398498],
            always_apply=True)
])

# mean=[0.45734706, 0.43338275, 0.40058118]
# std=[0.23965294, 0.23532275, 0.2398498]

# pytransform = transforms.Compose([
#     transforms.Resize(224),
#     transforms.Normalize(mean=mean, std=std),
#     transforms.ToTensor(),
# ])

class RoadSegmentationNode(Node):
    def __init__(self):
        super().__init__('road_segmentation')
        self.segmentation_publisher_ = self.create_publisher(Image, '/segmentation/road', 2)
        self.image_subscriber = self.create_subscription(Image, '/video/front_camera', self.segmentation_callback, 2)
        self.segmentation_timer = self.create_timer(.1, self.segmentation_callback)

        self.bridge = CvBridge()
        self.image = np.zeros([320, 240, 3], dtype=np.uint8)

        self.Unet = unet.UNet(in_chans=3, depth=3, layers=1, skip_connection=True)
        self.Unet.load_state_dict(torch.load('/workspaces/isaac_ros-dev/src/road_segmentation/models/checkpoints/unet.pkl'))
        self.sigmoid = nn.Sigmoid()
        self.Unet.eval()
        

        if torch.cuda.is_available():
            self.device = torch.device('cuda:0')
        else:
            self.device = torch.device('cpu')
        print('using device:', self.device)

        self.Unet.to(self.device)


    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)

    def segmentation_callback(self):
        with torch.no_grad():
            try:
                orig_frame = self.image.copy()
                orig_frame = cv2.resize(orig_frame, (224,224), interpolation = cv2.INTER_AREA)
                frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                frame = transform(image=frame)['image']
                # frame = pytransform(frame)
                # frame = torch.permute(frame, (2, 0, 1))
                frame = np.transpose(frame, (2, 0, 1))
                frame = torch.tensor(frame, dtype=torch.float32)
                frame = frame.unsqueeze(0).to(self.device)
                infer = self.Unet(frame).squeeze()
                infer = self.sigmoid(infer)

                infer = infer.detach().cpu().numpy()
                infer = np.where(infer >= .5, 0, 255).astype(np.uint8)
                    # Generate intermediate image; use morphological closing to keep parts together
                inter = cv2.morphologyEx(infer, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

                # Find largest contour in intermediate image
                cnts, _ = cv2.findContours(inter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                cnt = max(cnts, key=cv2.contourArea)

                # Output
                out = np.zeros(infer.shape, np.uint8)
                cv2.drawContours(out, [cnt], -1, 255, cv2.FILLED)
                out = cv2.bitwise_and(inter, out)

                out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
                
                added_image = cv2.addWeighted(orig_frame,0.5,out,0.5,0)

                ros_image = self.bridge.cv2_to_imgmsg(added_image)
                self.segmentation_publisher_.publish(ros_image)

            except Exception as e:
                print(e)



def main(args=None):
    rclpy.init(args=args)
    road_segmentation_node = RoadSegmentationNode()
    rclpy.spin(road_segmentation_node)
    road_segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

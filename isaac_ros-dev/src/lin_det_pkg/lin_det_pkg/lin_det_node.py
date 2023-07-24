#!/usr/bin/env python3 
import rclpy # enable use of ros 
from rclpy.node import Node # import node class 
import cv2
import numpy as np
#from matplotlib import pyplot as plt
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 




class LineFollowing(Node):
    
    def __init__(self):
        super().__init__("Line_det")
        self.direction_pr = float(50)
        self.publisher= self.create_publisher(Twist, 'line_tracking_vel', 10)
        #self.publisher2 = self.create_publisher(Image, 'line_tracking_image', 10)
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        #print("ok")
        try:
            frame= self.bridge.imgmsg_to_cv2(msg)
            _, directions = self.Tracking(frame)
            #ImtoRos = self.bridge.cv2_to_imgmsg(image)
            message = Twist()
            message.linear.x = 20.0
            message.angular.z = directions
            #self.publisher2.publish(ImtoRos)
            self.publisher.publish(message)
        except:
            pass
            #print("Not enough lines")
         
    def canny(self, frame):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Gaussian Blur
        blur_img = cv2.GaussianBlur(gray, (9, 9), cv2.BORDER_CONSTANT)
        
        #Canny
        canny = cv2.Canny(blur_img,100,200) 

        return canny 

    def region_of_interest(self, frame):
        height = frame.shape[0]
        polygons = np.array([[
        (0, height), (640, height), (320, 225)]
        ])
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(frame, mask)
        return masked_image

    def make_coordinates(self, frame, line_parameters):
        slope, intercept = line_parameters
        y1 = frame.shape[0]
        y2 = int(y1*(2/5))
        x1 = int((y1-intercept)/slope)
        x2 = int((y2-intercept)/slope)
        return np.array([x1, y1, x2, y2])

    def center_line(self, left_line, right_line):
        xbl, ybl, xtl, ytl = left_line  # xbl = x bottom, left line, xtl= x top, left line 
        xbr, ybr, xtr, ytr = right_line
        xcb = int((xbr-xbl)/2)
        xcb = xbl + xcb
        xct = int((xtr-xtl)/2)
        xct = xtl + xct
        yct = int(ytr)
        ycb = int(ybr)
        return np.array([xcb, ycb, xct, yct])

   
    def direction_provider(self,c_line_coord):

        xcb, ycb, xct, yct = c_line_coord
        #x_c = 320
        #slope_obj = 1.352175

        opp = xcb-xct 
        adj = ycb-yct
        angle = math.degrees(math.atan(opp/adj))


        if angle > 30:
            angle = 100

        elif angle < -30:
            angle = 0

        else:    
            angle = ((50/30)*angle) + 50 
            #print("angle in other numbers", angle)
    
        return angle 
    
    def direction_provider_s_l(self, line, fit_average):
        slope, intercept = fit_average
        x1, y1, x2, y2 = line
        if slope > 0:
            
            x1_obj = 473
            #x2_obj = 331
            slope_obj = 1.352175
            if x1 >=0.95*x1_obj and x1 <= 1.05*x1_obj and slope > 1.05*slope_obj:
                R = "You are driving to the left, turn right" 
            elif x1 >=0.95*x1_obj and x1 <= 1.05*x1_obj and slope < 0.95*slope_obj:
                R = "You are driving to the right, turn left" 
            elif slope >=0.95*slope_obj and slope <= 1.05*slope_obj and x1 > 1.05*x1_obj:
                R = "You are driving to much on left, turn right" 
            elif slope >=0.95*slope_obj and slope <= 1.05*slope_obj and x1 < 0.95*x1_obj:
                R = "You are driving to much on the right, turn left" 
            elif  slope > 1.05*slope_obj and x1 > 1.05*x1_obj:
                R = "while x > x_obj turn right, after turn right" 
            elif  slope < 0.95*slope_obj and x1 < 0.95*x1_obj:
                R = "while x < x_obj turn left, after turn left"
            elif slope < 0.95*slope_obj and x1 > 1.05*x1_obj:
                R  = "while x > x_obj turn right, after turn left"
            elif slope > 1.05*slope_obj and x1 < 0.95*x1_obj:
                R = "while x < x_obj turn left, after turn right"
            else:
                R = "driving ok"
                
        if slope < 0:
            
            x1_obj = 153
            slope_obj = -1.352175
            if x1 >=0.95*x1_obj and x1 <= 1.05*x1_obj and slope > 1.05*slope_obj:
                R = "You are driving to the left, turn right" 
            elif x1 >=0.95*x1_obj and x1 <= 1.05*x1_obj and slope < 0.95*slope_obj:
                R = "You are driving to the right, turn left" 
            elif slope >=0.95*slope_obj and slope <= 1.05*slope_obj and x1 > 1.05*x1_obj:
                R = "You are driving to much on left, turn right" 
            elif slope >=0.95*slope_obj and slope <= 1.05*slope_obj and x1 < 0.95*x1_obj:
                R = "You are driving to much on the right, turn left" 
            elif  slope > 1.05*slope_obj and x1 > 1.05*x1_obj:
                R = "while x > x_obj turn right, after turn right" 
            elif  slope < 0.95*slope_obj and x1 < 0.95*x1_obj:
                R = "while x < x_obj turn left, after turn left"
            elif slope < 0.95*slope_obj and x1 > 1.05*x1_obj:
                R  = "while x > x_obj turn right, after turn left"
            elif slope > 1.05*slope_obj and x1 < 0.95*x1_obj:
                R = "while x < x_obj turn left, after turn right"
            else:
                R = "driving ok"
        return R

    def average_slope_intercept(self, frame, lines):
        if lines is not None:
            left_fit = [] #coordinates of the average lines on the left
            right_fit = [] #coordinates of the average lines on the right
        
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                parameters = np.polyfit((x1,x2), (y1,y2),1)
                slope = parameters[0]
                intercept = parameters[1]
                if slope < -0.5:
                    left_fit.append((slope, intercept))

                elif slope > 0.5:
                    right_fit.append((slope, intercept)) 

            left_fit_average = np.average(left_fit, axis=0) #averages all slopes and all intercepts together
            right_fit_average = np.average(right_fit, axis=0)

            if np.size(left_fit_average) == 1 and np.size(right_fit_average) ==1:
                direction = 50
                return None, direction
            elif np.size(left_fit_average) == 1 and np.size(right_fit_average) ==2:
                right_line = self.make_coordinates(frame, right_fit_average)  
                direction =self.direction_provider_s_l(right_line, right_fit_average)
                return None, direction 
            elif np.size(left_fit_average) == 2 and np.size(right_fit_average) ==1:
                left_line = self.make_coordinates(frame, left_fit_average)  
                direction =self.direction_provider_s_l(left_line, left_fit_average)
                return None, direction
            else:
                right_line = self.make_coordinates(frame, right_fit_average)  
                left_line = self.make_coordinates(frame, left_fit_average)  
                c_line_coord = self.center_line(left_line, right_line)
                direction = self.direction_provider(c_line_coord)
                return np.array([left_line, right_line, c_line_coord]), direction
       
        
    def display_lines(self,frame, lines):
        line_image = np.zeros_like(frame)
        if lines is not None: 
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1,y1), (x2,y2), (255,0,0),10)
        central_line = cv2.line(line_image, (320,0),(320,480), (255,0,0),10)        
        return line_image
    
    def Tracking(self,frame):
        
        edges = self.canny(frame)
        masked_frame = self.region_of_interest(edges)
        lines = cv2.HoughLinesP(masked_frame, cv2.HOUGH_PROBABILISTIC, np.pi / 180, 50, minLineLength=100, maxLineGap=10) #og 170, line lenght og = 100 
        averaged_lines, directions = self.average_slope_intercept(frame, lines)
        #print("average lines", averaged_lines)
        #print("Directions", directions)
        if averaged_lines is not None:
            lines_image = self.display_lines(frame, averaged_lines)
            combo_lines_frame = cv2.addWeighted(frame, 0.8, lines_image, 1,1)
            directions = float(directions)
            if abs(directions-self.direction_pr) > 5:
                
                self.direction_pr = directions
            return combo_lines_frame, directions 
        
        # else:
        #     directions = float(50)
        #     lines_image = self.display_lines(frame, averaged_lines)
        #     combo_lines_frame = cv2.addWeighted(frame, 0.8, lines_image, 1,1)
        #     print("wrong direction")
        #     return combo_lines_frame, directions

            


def main(args=None):
    rclpy.init(args=args)    # strts ros2 communication, it is mandatory 
    node = LineFollowing()
    rclpy.spin(node)        # pauses program, keeps node running, so no shutdown
    rclpy.shutdown()

if __name__=="__main__":
    main()
    
    

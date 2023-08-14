import rclpy
from rclpy.node import Node

#from custom_message.msg import Wheels
from geometry_msgs.msg import Twist
import serial
import time
import sys
import math
PI = 3.1415
wheel_base = 1.6 # Meters
track_width = .87 # Meters
wheel_diameter = .33 # Meters

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)         # 1/timeout is the frequency at which the port is read
# ser.close()
# data ="0,0" # ser.readline().decode().strip()

class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")
        self.wheel_velocity_publisher = self.create_publisher(Twist, "odom_encoder", 1)
        self.timer_ = self.create_timer(0.2, self.publish_wheel_velocity)
        self.get_logger().info("publishing velocities")

        
        self.subscription = self.create_subscription(Twist,'/twist_mux/cmd_vel',self.listener_callback,1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        linear_velocity = round((msg.linear.x)/1.5,2)
       # print(linear_velocity)
        steering_angle = (msg.angular.z)
        steering_angle_int = int(steering_angle*61.0+512.0)
       # print(steering_angle_int)
        # print('listener called back')
        #message = str("0") + "," + str("512")
        message = str(linear_velocity) + "," + str(steering_angle_int) + "\n"
        # message = str(1) + "," + str(2) + "," + str(3) 
        #ser.write(bytes(message, 'utf-8'))
	#print(message)
        #time.sleep(.2)
        ser.write(bytes(message, 'utf-8'))
        #print(message)




    def publish_wheel_velocity(self):
        msg = Twist()
        # print(msg)
        #self.wheel_velocity_publisher.publish(msg)
        # print(ser.in_waiting)
        # print(ser.readline())
        # ser.open()
        # print(ser.in_waiting) 
        # print(ser.readline())
        # ser.close()
        # print("")
        # print(ser.in_waiting) 

        # print(ser.in_waiting) 
        # print("")
        # time.sleep(0.5)
        while ser.in_waiting > 0:
            # print(ser.readline())
            try:
                
                encoder_data = ser.readline()
        #        print(encoder_data)
                encoder_data = encoder_data.decode().strip()
                encoder_data_split = encoder_data.split(',') 
                encoder_data_list = [eval(encoder) for encoder in encoder_data_split]
                
                # # Fill X and Y linear velocities with wheel encoders
         #       print(encoder_data_list)

                right_wheel = float(encoder_data_list[0])*PI*wheel_diameter/60
                left_wheel = float(encoder_data_list[1])*PI*wheel_diameter/60
                msg.linear.x = (right_wheel + left_wheel)/2.0
                #msg.linear.y = float(encoder_data_list[1])*20/100

                # # Fill Z angular with Steering angle encoder
                steering_angle = -1.0*float(encoder_data_list[2])*(360/1023-180)
                angular_velocity = (msg.linear.x)/(track_width*math.tan(steering_angle))
                # print("Encoder Angular Velocity", (right_wheel - left_wheel)*.87)
                # print("Other angular velocity" ,angular_velocity)
                msg.angular.z = angular_velocity
                self.wheel_velocity_publisher.publish(msg)
                # # ser.reset_input_buffer()
       
            except Exception as e:
                print("an error has occured", e)
                # print(ser.readline())
        # message = str(1) + "," + str(2) + "," + str(3)
        #try:
        #    print(self.message) 
        #    ser.write(bytes(self.message, 'utf-8'))
        #    self.message = "0,0"
        #except Exception as e2:
        #    print("no cmdvel")



def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

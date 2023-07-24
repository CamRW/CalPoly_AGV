import rclpy
from rclpy.node import Node

#from custom_message.msg import Wheels
from geometry_msgs.msg import Twist
import serial
import time
import sys

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1) #, write_timeout=0.1)         # 1/timeout is the frequency at which the port is read
# ser.close()
# data ="0,0" # ser.readline().decode().strip()

class WheelsNode(Node):
    def __init__(self):
        super().__init__("wheel_velocity_publisher")
        self.wheel_velocity_publisher = self.create_publisher(Twist, "wheel_velocity", 1)
        self.timer_ = self.create_timer(0.2, self.publish_wheel_velocity)
        self.get_logger().info("publishing velocities")

        
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        linear_velocity = (msg.linear.x)
        steering_angle = (msg.angular.z)
        steering_angle_int = int(((steering_angle+1)/2)*1023)
        # print('listener called back')
        self.message = str("0") + "," + str("0")
        self.message = str(linear_velocity) + "," + str(steering_angle_int)
        # message = str(1) + "," + str(2) + "," + str(3) 
        # ser.write(bytes(message, 'utf-8'))



    def publish_wheel_velocity(self):
        msg = Twist()
        # print(msg)
        self.wheel_velocity_publisher.publish(msg)
        # print(ser.in_waiting)
        print(ser.readline())
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
                encoder_data = ser.readline().decode().strip()
                encoder_data_split = encoder_data.split(',') 
                encoder_data_list = [eval(encoder) for encoder in encoder_data_split]
                
                # # Fill X and Y linear velocities with wheel encoders
                # print(encoder_data_list)
                msg.linear.x = float(encoder_data_list[0])
                msg.linear.y = float(encoder_data_list[1])

                # # Fill Z angular with Steering angle encoder
                msg.angular.z = float(encoder_data_list[2])
                self.wheel_velocity_publisher.publish(msg)
                # # ser.reset_input_buffer()
       
            except Exception as e:
                print("an error has occured", e)
                # print(ser.readline())
        # message = str(1) + "," + str(2) + "," + str(3)
        # try:
        #     # print(self.message) 
        #     ser.write(bytes(self.message, 'utf-8'))
            
        # except Exception as e2:
        #     print(e2)



def main(args=None):
    rclpy.init(args=args)
    node = WheelsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
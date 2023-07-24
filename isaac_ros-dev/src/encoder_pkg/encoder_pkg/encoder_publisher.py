import rclpy
from rclpy.node import Node

#from custom_message.msg import Wheels
from geometry_msgs.msg import Twist
import serial
import time
import sys

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)         # 1/timeout is the frequency at which the port is read
# ser.reset_output_buffer()
# ser.reset_input_buffer()
data ="0,0" # ser.readline().decode().strip()


class WheelsNode(Node):
    def __init__(self):
        super().__init__("wheel_velocity_publisher")
        self.wheel_velocity_publisher = self.create_publisher(Twist, "wheel_velocity", 10)
        self.timer_ = self.create_timer(0.2, self.publish_wheel_velocity)
        self.get_logger().info("publishing velocities")

        # print("1")
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        linear_velocity = (msg.linear.x)
        steering_angle = (msg.angular.z)
        steering_angle_int = int(((steering_angle+1)/2)*1023)

        message = str(linear_velocity) + "," + str(steering_angle_int)
        times = self.get_clock().now()
        ser.reset_output_buffer()
        # ser.reset_input_buffer()
        # print(message, times, "size:", sys.getsizeof(message))
        # ser.write(bytes(message, 'utf-8'))
        # time.sleep(0.1)
        # print(ser.in_waiting)

        # ser.reset_output_buffer()

    def publish_wheel_velocity(self):
        msg = Twist()
        print(ser.in_waiting)
        while ser.in_waiting > 0:
            # print(ser.in_waiting)
            try:
                # print(ser.in_waiting)
                encoder_data = ser.readline().decode().strip()
                encoder_data_split = encoder_data.split(',') 
                # print(encoder_data_split)
                encoder_data_list = [eval(encoder) for encoder in encoder_data_split]
                
                # Fill X and Y linear velocities with wheel encoders
                msg.linear.x = float(encoder_data_list[0])
                msg.linear.y = float(encoder_data_list[1])

                # Fill Z angular with Steering angle encoder
                msg.angular.z = float(encoder_data_list[2])
                self.wheel_velocity_publisher.publish(msg)
                ser.reset_input_buffer()
       
            except Exception as e:
                print("an error has occured", e)
	    



def main(args=None):
    rclpy.init(args=args)
    node = WheelsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

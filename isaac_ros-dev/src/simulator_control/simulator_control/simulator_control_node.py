import rclpy
from rclpy.node import Node
from pyfirmata import Arduino
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


class SimulatorSubscriber(Node):

    def __init__(self):
        super().__init__('simulator_subscriber')
        # Create twist message subscriber
        self.controller_subscription = self.create_subscription(
            Twist,
            'vrx_vel',
            self.controller_callback,
            10)
        
        # Create timer and register callback to stop the vehicle if it doesn't receive a message in .5 seconds
        timer_period = 0.5 # Seconds
        self.timer = self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create object detection subscriber, object detection is done in the realsense_obj_det package
        self.obj_dec_subscription = self.create_subscription(Int16, 'object_det', self.obj_det_callback, 10)

        self.controller_subscription  # prevent unused variable warning
        self.obj_dec_subscription


        # Pyfirmata and pyserial serial port opening
        # Custom serial device names were given based on hardware identifiers
        # This allows the devices to be unplugged and plugged back in without changes to the serial device name
        # TODO, add retry period if serial fails
        self.board = Arduino('/dev/velocitySerial')
        self.steeringSerial = serial.Serial('/dev/steeringSerial', 9600)


        # Pin-setting for pyfirmata, see IBT2 motor driver for details on pinouts
        self.R_IS = 8
        self.R_EN = 2
        self.L_IS = 7
        self.L_EN = 4

        self.VEL_FWD_PWM = self.board.get_pin('d:5:p')
        self.VEL_RVS_PWM = self.board.get_pin('d:6:p')

        self.board.digital[self.R_IS].write(0)
        self.board.digital[self.L_IS].write(0)
        self.board.digital[self.R_EN].write(1)
        self.board.digital[self.L_EN].write(1)
        self.obj_det = 0


    def controller_callback(self, msg):
        # Reset the "kill-switch" timer when a message is received
        self.timer.reset()

        # Forward or reverse velocity logic and setting max/min values to 1.0 and -1.0
        linear_velocity = msg.linear.x
        if linear_velocity > 1.0:
            linear_velocity = 1.0
        if linear_velocity < -1.0:
            linear_velocity = -1.0
        
        # Scaling velocity down, motors have high max velocity
        linear_velocity = linear_velocity/4

        # Steering angle logic to take values from simulator chair or controller        
        steering_angle = msg.angular.z*50.0 + 50.0
        steering_angle = int(steering_angle)

        # print("Linear Vel: ", linear_velocity, type(linear_velocity))
        # print("Steering Angle: ", steering_angle, type(steering_angle))

        # All velocity commands should first check if there is an object before sending the command to the microcontrollers
        if self.obj_det == 0:
            #print("OBJ DET: 0")
            self.steeringSerial.write((str(steering_angle)+'\n').encode())

            # Forward
            if linear_velocity > 0:
                self.VEL_RVS_PWM.write(0)
                self.VEL_FWD_PWM.write(linear_velocity)
            # Reverse
            elif linear_velocity < 0:
                self.VEL_FWD_PWM.write(0)
                self.VEL_RVS_PWM.write(abs(linear_velocity))
            # If undefined, stop motors
            else:
                self.VEL_FWD_PWM.write(0)
                self.VEL_RVS_PWM.write(0)
        # If an object is detected or undefined behavior, stop motors
        else:
            #print("OBJ DET: 1")
            self.steeringSerial.write((str(steering_angle)+'\n').encode())
            self.VEL_FWD_PWM.write(0)
            self.VEL_RVS_PWM.write(0)

    # Object detection callback
    def obj_det_callback(self, msg):
        self.obj_det = msg.data

    # "Kill-switch" timer callback
    def timer_callback(self):
        self.VEL_FWD_PWM.write(0)
        self.VEL_RVS_PWM.write(0)




def main(args=None):
    rclpy.init(args=args)

    simulator_subscriber = SimulatorSubscriber()

    rclpy.spin(simulator_subscriber)

    # When this node is destroyed or dies, stop the vehicle
    simulator_subscriber.VEL_FWD_PWM.write(0)
    simulator_subscriber.VEL_RVS_PWM.write(0)
    simulator_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

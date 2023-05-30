import rclpy
from rclpy.node import Node
from pyfirmata import Arduino
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


class SimulatorSubscriber(Node):

    def __init__(self):
        super().__init__('simulator_subscriber')
        self.controller_subscription = self.create_subscription(
            Twist,
            'vrx_vel',
            self.controller_callback,
            10)
        timer_period = 0.5 # Seconds
        self.timer = self.timer = self.create_timer(timer_period, self.timer_callback)
        self.obj_dec_subscription = self.create_subscription(Int16, 'object_det', self.obj_det_callback, 10)
        self.controller_subscription  # prevent unused variable warning
        self.obj_dec_subscription

        self.board = Arduino('/dev/velocitySerial')
        self.steeringSerial = serial.Serial('/dev/steeringSerial', 9600)

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
        self.timer.reset()
        linear_velocity = msg.linear.x
        if linear_velocity > 1.0:
            linear_velocity = 1.0
        if linear_velocity < -1.0:
            linear_velocity = -1.0
        linear_velocity = linear_velocity/4
        #steering_angle = int(msg.angular.z) from chair [0,50,100]
        steering_angle = msg.angular.z*50.0 + 50.0
        steering_angle = int(steering_angle)
        print("Linear Vel: ", linear_velocity, type(linear_velocity))
        print("Steering Angle: ", steering_angle, type(steering_angle))

        if self.obj_det == 0:
            print("OBJ DET: 0")
            self.steeringSerial.write((str(steering_angle)+'\n').encode())
            if linear_velocity > 0:
                self.VEL_RVS_PWM.write(0)
                self.VEL_FWD_PWM.write(linear_velocity)
            elif linear_velocity < 0:
                self.VEL_FWD_PWM.write(0)
                self.VEL_RVS_PWM.write(abs(linear_velocity))
            else:
                self.VEL_FWD_PWM.write(0)
                self.VEL_RVS_PWM.write(0)
        else:
            print("OBJ DET: 1")
            self.steeringSerial.write((str(steering_angle)+'\n').encode())
            self.VEL_FWD_PWM.write(0)
            self.VEL_RVS_PWM.write(0)

    def obj_det_callback(self, msg):
        self.obj_det = msg.data

    def timer_callback(self):
        self.VEL_FWD_PWM.write(0)
        self.VEL_RVS_PWM.write(0)




def main(args=None):
    rclpy.init(args=args)

    simulator_subscriber = SimulatorSubscriber()

    rclpy.spin(simulator_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_subscriber.VEL_FWD_PWM.write(0)
    simulator_subscriber.VEL_RVS_PWM.write(0)
    simulator_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

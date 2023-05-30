from gps_package import GPS_Extract
import serial
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        self.coords_publisher = self.create_publisher(NavSatFix, '/nav/raw/fix', 10)
        self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, '/nav/raw/twistcov', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.gps_serial = serial.Serial('/dev/ttyACM1', 115200)
        except:
            print("Serial could not connect")

            

    def timer_callback(self):
        raw_taip = self.gps_serial.readline().decode("utf-8")
        #print("TAIP", raw_taip)
        data = GPS_Extract.extract_fields(raw_taip)
        #print(data)

        if len(data)>0:

            coords = data[0:2]
            #print("COORDS", coords)
            speed = float(data[2])
            heading = float(data[3]) # Currently unused

            coords_msg = NavSatFix()
            twist_msg = TwistWithCovarianceStamped()
            coords_msg.header.frame_id = 'GPS_coords'
            twist_msg.header.frame_id = 'GPS_twist'

            coords_msg.header.stamp = self._clock.now().to_msg()
            twist_msg.header.stamp = coords_msg.header.stamp


            coords_msg.latitude = coords[0]
            coords_msg.longitude = coords[1]

            twist_msg.twist._twist._linear.x = speed
            twist_msg.twist._twist._angular.z = 0.0

            self.twist_publisher.publish(twist_msg)
            self.coords_publisher.publish(coords_msg)

        else:
            pass
        

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSNode()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

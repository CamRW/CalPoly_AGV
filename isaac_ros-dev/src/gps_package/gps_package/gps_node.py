from gps_package import GPS_Extract
import serial
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node

# ROS2 GPS node for publishing NavSatFix messages from
# Airlink MP70 wireless module

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        
        # Create publishers, currently TwistWithCov is unused
        self.coords_publisher = self.create_publisher(NavSatFix, '/nav/raw/fix', 10)
        #self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, '/nav/raw/twistcov', 10)
        
        # Publishing GPS messages every .2 seconds
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Open serial port to modem
        try:
            self.gps_serial = serial.Serial('/dev/ttyACM0', 115200)
        except:
            print("Serial could not connect")

            
    # Callback used to publish message
    def timer_callback(self):
        # Get raw message
        raw_taip = self.gps_serial.readline().decode("utf-8")

        # Extract gps data into its relative fields
        data = GPS_Extract.extract_fields(raw_taip)
        print(data)

        if len(data)>0:


            # Coordinates are first two elements
            coords = data[0:2]

            # Currently unused fields, TAIP-PV message gives velocity and heading
            # values, but they are not used in this implementation
            speed = float(data[2])
            heading = float(data[3])
            print(heading)

            # Create message objects and give them frame ids
            coords_msg = NavSatFix()
            #twist_msg = TwistWithCovarianceStamped()
            coords_msg.header.frame_id = 'GPS_coords'
            #twist_msg.header.frame_id = 'GPS_twist'
            
            # Add timestamp to message
            coords_msg.header.stamp = self._clock.now().to_msg()
            #twist_msg.header.stamp = coords_msg.header.stamp

            # Fill message fields
            coords_msg.latitude = coords[0]/1000.0
            coords_msg.longitude = coords[1]/1000.0

            #twist_msg.twist._twist._linear.x = speed
            #twist_msg.twist._twist._angular.z = 0.0

            #self.twist_publisher.publish(twist_msg)
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

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import serial
from imus.IMUdriver import *

class imus(Node):
    def __init__(self):
        super().__init__('imus')
        self.get_logger().info('imus node started')

        # Parameters
        self.declare_parameter('port', "/dev/ttyUSB0")  # "/dev/ttyAMA0"
        self.declare_parameter('topic_name', "IMU_comp")

        port = self.get_parameter('port').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Publishers
        self.IMU_publisher = self.create_publisher(Twist, '/sensor/' + topic_name, 10)

        # Variables
        config_directory_path = get_package_share_directory('imus')+"/cfg"
        self.IMU = IMU(port=port, parameter_path=config_directory_path)

    def timer_callback(self):
        self.IMU.update_once()
        euler_angles = self.IMU.euler_angles

        self.get_logger().info('Yaw: {}, Pitch: {}, Roll: {}'.format(euler_angles[0], euler_angles[1], euler_angles[2]))
        msg = Twist()
        msg.angular.x = euler_angles[0]
        msg.angular.y = euler_angles[1]
        msg.angular.z = euler_angles[2] # Useless in our case
        self.IMU_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = imus()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

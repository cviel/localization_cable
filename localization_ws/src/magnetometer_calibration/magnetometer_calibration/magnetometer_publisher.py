import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import MagneticField
import time
from imus.IMUdriver import IMU
from magnetometer_calibration.compute_calibration_parameters import compute_calibration_parameters
import os
import numpy as np

class MagnetometerPublisher(Node):
    def __init__(self):
        super().__init__('magnetometer_publisher')
        self.get_logger().info('Magnetometer publisher node started')

        # Parameters
        self.declare_parameter('port', "/dev/ttyAMA0")
        port = self.get_parameter('port').get_parameter_value().string_value
        
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Publishers
        self.publisher_ = self.create_publisher(MagneticField, 'magnetic_data', 10)

        # Variables
        self.imu = IMU(port=port, parameter_path="##CALIBRATING##")
        self.imu.serial.write(("#osrt").encode("utf-8"))  # Ask for raw mag
        
        # Define the center and compensation matrix path
        self.center_path = os.path.join(get_package_share_directory('imus'), 'cfg', 'magn_ellipsoid_center.txt')
        self.comp_path = os.path.join(get_package_share_directory('imus'), 'cfg', 'magn_ellipsoid_transform.txt')
        
        self.mag_data = []
        
    def timer_callback(self):
        self.imu.update_once()
        if (self.imu.mag[0] + self.imu.mag[1] + self.imu.mag[2] != 0 and self.imu.mag_updated):
            mag_msg = MagneticField()
            mag_msg.magnetic_field.x = self.imu.mag[0]
            mag_msg.magnetic_field.y = self.imu.mag[1]
            mag_msg.magnetic_field.z = self.imu.mag[2]
            
            self.publisher_.publish(mag_msg)
            
            self.get_logger().info(f"{self.imu.mag[0]} {self.imu.mag[1]} {self.imu.mag[2]}")
            self.mag_data.append([self.imu.mag[0], self.imu.mag[1], self.imu.mag[2]])
            self.imu.mag_updated = False

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # input("Overwrite previous calibration data? Press Enter to continue, Ctrl+C to cancel.")

        # node.file.close()

        center, comp = compute_calibration_parameters(np.array(node.mag_data))
        node.get_logger().info(f"Center of the ellipsoid: {center}")
        node.get_logger().info(f"Compensation matrix: {comp}")

        np.savetxt(node.center_path, center, fmt='%f')
        np.savetxt(node.comp_path, comp, fmt='%f')

        node.get_logger().info(f"Center and compensation matrix saved to files.")
        node.get_logger().info(f" ")
        node.get_logger().info(f" FIN ")
        node.get_logger().info(f" ")    
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

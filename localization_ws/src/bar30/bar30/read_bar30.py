import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from . import ms5837

class bar30(Node):
    def __init__(self):
        super().__init__('bar30')
        self.get_logger().info('bar30 node started')

        # Parameters
        self.declare_parameter('pressure_calibration', 100000.0)
        self.pressure_calibration = self.get_parameter('pressure_calibration').get_parameter_value().double_value
        
        self.sensor = ms5837.MS5837_30BA()
        if not self.sensor.init():
            self.get_logger().info('Sensor could not be initialized')
            exit(1)
        else:
            self.get_logger().info('Sensor initialized')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Publishers
        self.z_publisher = self.create_publisher(Float32, '/sensor/z', 10)

    def timer_callback(self):
        if self.sensor.read():
            rho, g = 1000, 9.81
            
            msg = Float32()
            msg.data = (self.sensor.pressure(ms5837.UNITS_Pa) - self.pressure_calibration) / (rho * g)
            self.z_publisher.publish(msg)
            # self.get_logger().info('z: %0.1f m' % msg.data)
        

def main(args=None):
    rclpy.init(args=args)

    bar30_node = bar30()

    rclpy.spin(bar30_node)

    bar30_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
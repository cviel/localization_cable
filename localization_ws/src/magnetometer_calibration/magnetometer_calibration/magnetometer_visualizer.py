import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MagnetometerVisualizer(Node):
    def __init__(self):
        super().__init__('magnetometer_visualizer')
        self.get_logger().info('Magnetometer visualizer node started')
        
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        # Subscribers
        self.subscription = self.create_subscription(
            MagneticField,
            'magnetic_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Variables
        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')


        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z

        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

    def timer_callback(self):
        if self.x_data and self.y_data and self.z_data:
            self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.scatter(self.x_data, self.y_data, self.z_data, c='r', marker='o')

        self.ax.set_xlabel('Mag X')
        self.ax.set_ylabel('Mag Y')
        self.ax.set_zlabel('Mag Z')

        plt.title('Real-time Magnetometer Measurements')
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

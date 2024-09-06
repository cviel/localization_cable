import rclpy
from rclpy.node import Node
from msgs.msg import Interval
from interval_computation.tools import draw_cube_mtpltlb # type: ignore
import codac as cdc
import matplotlib.pyplot as plt

class Simulation(Node):
    def __init__(self):
        super().__init__('simulation')
        self.get_logger().info('Simulation node started')

        # Parameters
        self.declare_parameter('L', 1.0)
        self.declare_parameter('z_offset', 0.0)
        
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscriptions
        self.x_subscriber = self.create_subscription(Interval, '/interval/x', self.x_callback, 10)
        self.y_subscriber = self.create_subscription(Interval, '/interval/y', self.y_callback, 10)
        self.z_subscriber = self.create_subscription(Interval, '/interval/z', self.z_callback, 10)
        self.x_mg_subscriber = self.create_subscription(Interval, '/interval/x_mg', self.x_mg_callback, 10)
        self.y_mg_subscriber = self.create_subscription(Interval, '/interval/y_mg', self.y_mg_callback, 10)
        self.z_mg_subscriber = self.create_subscription(Interval, '/interval/z_mg', self.z_mg_callback, 10)

        # Variables
        self.x = cdc.Interval()
        self.y = cdc.Interval()
        self.z = cdc.Interval()
        self.x_mg = cdc.Interval()
        self.y_mg = cdc.Interval()
        self.z_mg = cdc.Interval()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def x_callback(self, msg):
        self.x = cdc.Interval(msg.lb, msg.ub)
    
    def y_callback(self, msg):
        self.y = cdc.Interval(msg.lb, msg.ub)
    
    def z_callback(self, msg):
        self.z = cdc.Interval(msg.lb, msg.ub)
    
    def x_mg_callback(self, msg):
        self.x_mg = cdc.Interval(msg.lb, msg.ub)
    
    def y_mg_callback(self, msg):
        self.y_mg = cdc.Interval(msg.lb, msg.ub)
    
    def z_mg_callback(self, msg):
        self.z_mg = cdc.Interval(msg.lb, msg.ub)
    
    def timer_callback(self):
        plt.cla()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim(-self.L, self.L)
        self.ax.set_ylim(self.L, -self.L)
        self.ax.set_zlim(self.L, 0)
        
        # ROV box
        draw_cube_mtpltlb(self.ax, cdc.IntervalVector([self.x, self.y, self.z]), 'r')

        # Masse glissante box
        draw_cube_mtpltlb(self.ax, cdc.IntervalVector([self.x_mg, self.y_mg, self.z_mg]), 'b')

        # Ligne entre O et masse glissante
        self.ax.plot([0, self.x_mg.mid()], [0, self.y_mg.mid()], [-self.z_offset, self.z_mg.mid()], 'g')

        # Ligne entre masse glissante et ROV
        self.ax.plot([self.x_mg.mid(), self.x.mid()], [self.y_mg.mid(), self.y.mid()], [self.z_mg.mid(), self.z.mid()], 'g')

        plt.pause(0.0001) # To draw the plot

def main(args=None):
    rclpy.init(args=args)

    simulation = Simulation()

    rclpy.spin(simulation)

    simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from msgs.msg import Interval

from .contractors import *
from .tools import *

class ROV_pos(Node):
    def __init__(self):
        super().__init__('ROV_pos')
        self.get_logger().info('ROV_pos node started')

        # Parameters
        self.declare_parameter('sivia_eps', 0.278)
        self.declare_parameter('L', 1.0)
        self.declare_parameter('z_offset', 0.0)
        self.declare_parameter('IMU_precision', 0.053)
        self.declare_parameter('L_precision', 0.05)
        self.declare_parameter('z_precision', 0.05)

        self.sivia_eps = self.get_parameter('sivia_eps').get_parameter_value().double_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self.IMU_precision = self.get_parameter('IMU_precision').get_parameter_value().double_value
        self.L_precision = self.get_parameter('L_precision').get_parameter_value().double_value
        self.z_precision = self.get_parameter('z_precision').get_parameter_value().double_value

        self.timer_period = 0.25
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscriptions
        self.IMU_comp_subscription = self.create_subscription(Twist, '/sensor/IMU_comp', self.IMU_comp_callback, 10)
        self.IMU_ROV_subscription = self.create_subscription(Twist, '/sensor/IMU_ROV', self.IMU_ROV_callback, 10)
        self.z_subscription = self.create_subscription(Float32, '/sensor/z', self.z_callback, 10)

        # Publishers
        # ROV position
        self.x_publisher = self.create_publisher(Interval, '/interval/x', 10)
        self.y_publisher = self.create_publisher(Interval, '/interval/y', 10)
        self.z_publisher = self.create_publisher(Interval, '/interval/z', 10)
        # Masse glissante
        self.x_mg_publisher = self.create_publisher(Interval, '/interval/x_mg', 10)
        self.y_mg_publisher = self.create_publisher(Interval, '/interval/y_mg', 10)
        self.z_mg_publisher = self.create_publisher(Interval, '/interval/z_mg', 10)
        # Empty intervals
        self.x_empty_publisher = self.create_publisher(Interval, '/interval/x_empty', 10)
        self.y_empty_publisher = self.create_publisher(Interval, '/interval/y_empty', 10)

        # Variables
        self.alpha = 0
        self.beta = 0
        self.mu = 0
        self.eta = 0
        self.z = 0

    def IMU_comp_callback(self, msg):
        yaw, pitch = msg.angular.x * np.pi / 180, msg.angular.y * np.pi / 180

	# conversion angle IMU PC vers angles repere global
        self.alpha = (np.pi/2 + pitch) * np.cos(yaw)
        self.mu = (np.pi/2 + pitch) * np.sin(yaw)

        # self.get_logger().info('alpha: {}, mu: {}'.format(self.alpha, self.mu))
    
    def IMU_ROV_callback(self, msg):
        yaw, pitch = msg.angular.x * np.pi / 180, msg.angular.y * np.pi / 180

	# conversion angle IMU ROV vers angles repere global
	yaw, pitch = yaw - np.pi, - pitch
        self.beta = - (np.pi/2 + pitch) * np.cos(yaw)
        self.eta = - (np.pi/2 + pitch) * np.sin(yaw)

        # self.get_logger().info('beta: {}, eta: {}'.format(self.beta, self.eta))

    def z_callback(self, msg):
        self.z = msg.data + self.z_offset
        # self.z = 0.2 + self.z_offset
        # self.get_logger().info('z: {}'.format(self.z))

    def timer_callback(self):
        def solve(X):
            x, z, y, alpha, beta, mu, eta, L, a1, a2, a3, a4, l1, l2, l1x, l1y, l2x, l2y, x_m_gliss, y_m_gliss, z_m_gliss = X

            # Initialisation des contractor networks
            cn = cdc.ContractorNetwork()

            # Ajout des contracteurs
            cn.add(ctc_a1a2, [alpha, beta, mu, eta, a1, a2])
            cn.add(ctc_a3a4, [alpha, beta, mu, eta, a3, a4])
            cn.add(ctc_l1, [z, alpha, beta, mu, eta, a1, a2, a3, a4, L, l1])
            cn.add(ctc_l2, [z, alpha, beta, mu, eta, a1, a2, a3, a4, L, l2])
            cn.add(ctc_L, [l1, l2, L])

            cn.add(ctc_l1l2xy, [a1, a2, a3, a4, l1, l2, l1x, l1y, l2x, l2y])
            cn.add(ctc_xy, [alpha, beta, mu, eta, l1x, l1y, l2x, l2y, x, y])
            cn.add(ctc_m_gliss, [alpha, mu, l1x, l1y, x_m_gliss, y_m_gliss, z_m_gliss])

            # Résolution des contractor networks
            cn.contract(False)
        
        # Inputs
        alpha = cdc.Interval(self.alpha).inflate(self.IMU_precision)
        beta = cdc.Interval(self.beta).inflate(self.IMU_precision)
        mu = cdc.Interval(self.mu).inflate(self.IMU_precision)
        eta = cdc.Interval(self.eta).inflate(self.IMU_precision)
        L = cdc.Interval(self.L).inflate(self.L_precision)
        z = cdc.Interval(self.z).inflate(self.z_precision)

        # Empty inputs
        alpha_empty = cdc.Interval(self.alpha)
        beta_empty = cdc.Interval(self.beta)
        mu_empty = cdc.Interval(self.mu)
        eta_empty = cdc.Interval(self.eta)
        L_empty = cdc.Interval(self.L)
        z_empty = cdc.Interval(self.z)

        # Intermediate variables
        a1, a2, a3, a4 = cdc.IntervalVector(4, [0, cdc.oo])
        l1x, l1y, l2x, l2y = cdc.IntervalVector(4, [0, L.ub()])

        # Outputs
        l1, l2 = cdc.IntervalVector(2, [0, L.ub()])
        x, y = cdc.IntervalVector(2, [-L.ub(), L.ub()])
        x_mg = cdc.Interval(-L.ub(), L.ub())
        y_mg = cdc.Interval(-L.ub(), L.ub())
        z_mg = cdc.Interval(0, L.ub())

        # Empty outputs (with 0-diameter intervals)
        x_empty, y_empty = cdc.IntervalVector(2, [-L.ub(), L.ub()])


        # Solve with SIVIA
        X0 = cdc.IntervalVector([x, z, y, alpha, beta, mu, eta, L, a1, a2, a3, a4, l1, l2, l1x, l1y, l2x, l2y, x_mg, y_mg, z_mg])

        X0_empty = cdc.IntervalVector(X0) # Copy
        X0_empty[0] = x_empty
        X0_empty[1] = z_empty
        X0_empty[2] = y_empty
        X0_empty[3] = alpha_empty
        X0_empty[4] = beta_empty
        X0_empty[5] = mu_empty
        X0_empty[6] = eta_empty
        X0_empty[7] = L_empty

        solve(X0_empty)

        _, res_y = SIVIA_cn(X0, solve, self.sivia_eps, draw_boxes=False)

        if len(res_y) == 0:
            self.get_logger().info('No solution found')
            return
        
        # On calcule la boîte englobante du résultat
        X0 = union_list(res_y)

        x, z, y, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, x_mg, y_mg, z_mg = X0
        x_empty, _, y_empty, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _= X0_empty

        # Remove z_offset to output
        z = z - self.z_offset
        z_mg = z_mg - self.z_offset

        self.get_logger().info('x = {:.3f} (diam = {:.3f}), y = {:.3f} (diam = {:.3f}), z = {:.2f} (diam = {:.2f})'.format(x.mid(), x.diam(), y.mid(), y.diam(), z.mid(), z.diam()))


        # Publish
        x_msg = Interval()
        x_msg.lb, x_msg.ub = x.lb(), x.ub()
        self.x_publisher.publish(x_msg)

        y_msg = Interval()
        y_msg.lb, y_msg.ub = y.lb(), y.ub()
        self.y_publisher.publish(y_msg)

        z_msg = Interval()
        z_msg.lb, z_msg.ub = z.lb(), z.ub()
        self.z_publisher.publish(z_msg)

        x_mg_msg = Interval()
        x_mg_msg.lb, x_mg_msg.ub = x_mg.lb(), x_mg.ub()
        self.x_mg_publisher.publish(x_mg_msg)

        y_mg_msg = Interval()
        y_mg_msg.lb, y_mg_msg.ub = y_mg.lb(), y_mg.ub()
        self.y_mg_publisher.publish(y_mg_msg)

        z_mg_msg = Interval()
        z_mg_msg.lb, z_mg_msg.ub = z_mg.lb(), z_mg.ub()
        self.z_mg_publisher.publish(z_mg_msg)

        x_empty_msg = Interval()
        x_empty_msg.lb, x_empty_msg.ub = x_empty.lb(), x_empty.ub()
        self.x_empty_publisher.publish(x_empty_msg)

        y_empty_msg = Interval()
        y_empty_msg.lb, y_empty_msg.ub = y_empty.lb(), y_empty.ub()
        self.y_empty_publisher.publish(y_empty_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ROV_pos()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class TesteOdom(Node):

    def __init__(self):
        super().__init__('teste_odom')

        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Teste de odometria iniciado.'
        )

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw
        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        yaw_deg = math.degrees(yaw)

        self.get_logger().info(
            f'x = {x:.3f} m | '
            f'y = {y:.3f} m | '
            f'yaw = {yaw_deg:.1f}°'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TesteOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class PercursoQuadrado(Node):

    def __init__(self):
        super().__init__('percurso_quadrado')

        # ==========================================
        # CONFIGURAÇÕES DO TESTE
        # ==========================================

        # Quadrado de 40 cm
        self.side_length = 0.40

        # Velocidade para frente
        self.linear_speed = 0.12

        # Velocidade máxima de giro
        self.max_angular_speed = 0.8

        # Controle proporcional do giro
        self.kp_angular = 2.0

        # Tolerâncias
        self.distance_tolerance = 0.01
        self.angle_tolerance = math.radians(2.0)

        # ==========================================
        # ESTADO DO ROBÔ
        # ==========================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False

        # Estados:
        # FORWARD
        # TURN
        # FINISHED
        self.state = 'WAITING'

        self.side = 0

        self.start_x = 0.0
        self.start_y = 0.0

        self.initial_yaw = 0.0
        self.target_yaw = 0.0

        # ==========================================
        # ROS
        # ==========================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Controle a 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Controlador do quadrado iniciado.'
        )

        self.get_logger().info(
            'Aguardando odometria...'
        )

    # ==========================================
    # ODOMETRIA
    # ==========================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        if not self.odom_received:

            self.odom_received = True

            self.start_x = self.x
            self.start_y = self.y

            self.initial_yaw = self.yaw

            self.state = 'FORWARD'

            self.get_logger().info(
                'Odometria recebida!'
            )

            self.get_logger().info(
                'Iniciando lado 1.'
            )

    # ==========================================
    # ENVIA VELOCIDADE
    # ==========================================

    def send_velocity(self, linear, angular):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    # ==========================================
    # LOOP DE CONTROLE
    # ==========================================

    def control_loop(self):

        if not self.odom_received:
            return

        # --------------------------------------
        # TERMINOU
        # --------------------------------------

        if self.state == 'FINISHED':

            self.send_velocity(0.0, 0.0)

            return

        # --------------------------------------
        # ANDANDO RETO
        # --------------------------------------

        if self.state == 'FORWARD':

            dx = self.x - self.start_x
            dy = self.y - self.start_y

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            # Terminou o lado
            if distance >= (
                self.side_length -
                self.distance_tolerance
            ):

                self.send_velocity(
                    0.0,
                    0.0
                )

                self.get_logger().info(
                    f'Lado {self.side + 1} concluído: '
                    f'{distance:.3f} m'
                )

                # Próximo ângulo:
                # +90 graus
                self.target_yaw = normalize_angle(
                    self.initial_yaw +
                    (self.side + 1) *
                    math.pi / 2.0
                )

                self.state = 'TURN'

                self.get_logger().info(
                    'Girando 90 graus...'
                )

                return

            # ----------------------------------
            # CORREÇÃO DA RETA
            # ----------------------------------

            expected_yaw = normalize_angle(
                self.initial_yaw +
                self.side *
                math.pi / 2.0
            )

            yaw_error = normalize_angle(
                expected_yaw -
                self.yaw
            )

            angular_correction = (
                2.0 * yaw_error
            )

            angular_correction = max(
                -0.25,
                min(
                    0.25,
                    angular_correction
                )
            )

            self.send_velocity(
                self.linear_speed,
                angular_correction
            )

        # --------------------------------------
        # GIRANDO
        # --------------------------------------

        elif self.state == 'TURN':

            angle_error = normalize_angle(
                self.target_yaw -
                self.yaw
            )

            # Terminou o giro
            if abs(angle_error) <= self.angle_tolerance:

                self.send_velocity(
                    0.0,
                    0.0
                )

                self.side += 1

                # Terminou os quatro lados
                if self.side >= 4:

                    self.state = 'FINISHED'

                    self.get_logger().info(
                        'QUADRADO CONCLUÍDO!'
                    )

                    return

                # Novo ponto inicial
                self.start_x = self.x
                self.start_y = self.y

                self.state = 'FORWARD'

                self.get_logger().info(
                    f'Iniciando lado {self.side + 1}.'
                )

                return

            # Controle proporcional do giro
            angular = (
                self.kp_angular *
                angle_error
            )

            angular = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    angular
                )
            )

            # Mantém velocidade mínima perto do alvo
            if (
                abs(angular) < 0.15
                and abs(angle_error) >
                self.angle_tolerance
            ):

                angular = math.copysign(
                    0.15,
                    angle_error
                )

            self.send_velocity(
                0.0,
                angular
            )


def main(args=None):

    rclpy.init(args=args)

    node = PercursoQuadrado()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.send_velocity(
            0.0,
            0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

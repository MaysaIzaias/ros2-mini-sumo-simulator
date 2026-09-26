import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry


class ControladorPercurso(Node):

    def __init__(self):

        super().__init__('controlador_percurso')


        # =====================================
        # PARÂMETROS DO CONTROLADOR
        # =====================================

        self.declare_parameter(
            'max_linear',
            0.40
        )

        self.declare_parameter(
            'max_angular',
            1.0
        )

        self.declare_parameter(
            'kp_linear',
            0.8
        )

        self.declare_parameter(
            'kp_angular',
            2.0
        )

        self.declare_parameter(
            'tolerancia_posicao',
            0.08
        )

        self.declare_parameter(
            'tolerancia_angulo',
            0.20
        )


        self.max_linear = self.get_parameter(
            'max_linear'
        ).value

        self.max_angular = self.get_parameter(
            'max_angular'
        ).value

        self.kp_linear = self.get_parameter(
            'kp_linear'
        ).value

        self.kp_angular = self.get_parameter(
            'kp_angular'
        ).value

        self.tolerancia_posicao = self.get_parameter(
            'tolerancia_posicao'
        ).value

        self.tolerancia_angulo = self.get_parameter(
            'tolerancia_angulo'
        ).value


        # =====================================
        # PERCURSO
        # =====================================

        self.waypoints = [

            (1.0, 0.0),

            (1.0, 1.0),

            (0.0, 1.0),

            (0.0, 0.0)

        ]


        self.indice = 0


        # Estado atual
        self.x = 0.0

        self.y = 0.0

        self.theta = 0.0

        self.recebeu_odom = False


        # =====================================
        # ROS
        # =====================================

        self.publisher = self.create_publisher(

            TwistStamped,

            '/diff_drive_controller/cmd_vel',

            10

        )


        self.subscription = self.create_subscription(

            Odometry,

            '/diff_drive_controller/odom',

            self.odom_callback,

            10

        )


        self.timer = self.create_timer(

            0.05,

            self.control_loop

        )


        self.get_logger().info(
            'Controlador de percurso iniciado.'
        )


    # =========================================
    # ODOMETRIA
    # =========================================

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


        self.theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )


        self.recebeu_odom = True


    # =========================================
    # NORMALIZAÇÃO DE ÂNGULO
    # =========================================

    def normalizar_angulo(self, angulo):

        while angulo > math.pi:

            angulo -= 2.0 * math.pi


        while angulo < -math.pi:

            angulo += 2.0 * math.pi


        return angulo


    # =========================================
    # PUBLICAR COMANDO
    # =========================================

    def enviar_comando(
        self,
        linear,
        angular
    ):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        msg.twist.linear.x = float(linear)

        msg.twist.angular.z = float(angular)


        self.publisher.publish(msg)


    # =========================================
    # LOOP PRINCIPAL
    # =========================================

    def control_loop(self):

        if not self.recebeu_odom:

            return


        if self.indice >= len(self.waypoints):

            self.enviar_comando(
                0.0,
                0.0
            )

            return


        alvo_x, alvo_y = (
            self.waypoints[self.indice]
        )


        dx = alvo_x - self.x

        dy = alvo_y - self.y


        distancia = math.sqrt(
            dx * dx +
            dy * dy
        )


        # Chegou ao ponto
        if distancia < self.tolerancia_posicao:

            self.get_logger().info(

                f'Waypoint {self.indice + 1} atingido: '
                f'({alvo_x:.2f}, {alvo_y:.2f})'

            )

            self.indice += 1

            self.enviar_comando(
                0.0,
                0.0
            )

            return


        angulo_desejado = math.atan2(
            dy,
            dx
        )


        erro_angular = self.normalizar_angulo(

            angulo_desejado -
            self.theta

        )


        velocidade_angular = (

            self.kp_angular *
            erro_angular

        )


        velocidade_angular = max(

            -self.max_angular,

            min(
                self.max_angular,
                velocidade_angular
            )

        )


        # Se estiver apontando muito errado,
        # primeiro gira.
        if abs(erro_angular) > self.tolerancia_angulo:

            velocidade_linear = 0.0

        else:

            velocidade_linear = (

                self.kp_linear *
                distancia

            )


            velocidade_linear = min(

                self.max_linear,

                velocidade_linear

            )


        self.enviar_comando(

            velocidade_linear,

            velocidade_angular

        )


def main(args=None):

    rclpy.init(args=args)

    node = ControladorPercurso()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.enviar_comando(
            0.0,
            0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

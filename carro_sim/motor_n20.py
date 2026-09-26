import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MotorN20(Node):

    def __init__(self):

        super().__init__('motor_n20')

        # ==========================================================
        # PARÂMETROS FÍSICOS DO ROBÔ
        # ==========================================================

        self.declare_parameter('wheel_radius', 0.016)

        self.declare_parameter(
            'wheel_separation',
            0.121
        )

        # ==========================================================
        # MOTOR N20
        # ==========================================================

        # Velocidade no eixo DEPOIS da redução.
        self.declare_parameter(
            'motor_rpm',
            750.0
        )

        # Torque de stall no eixo DEPOIS da redução.
        self.declare_parameter(
            'stall_torque',
            0.009
        )

        # Encoder depois da redução.
        self.declare_parameter(
            'encoder_ppr',
            140
        )

        self.wheel_radius = (
            self.get_parameter('wheel_radius')
            .value
        )

        self.wheel_separation = (
            self.get_parameter('wheel_separation')
            .value
        )

        self.motor_rpm = (
            self.get_parameter('motor_rpm')
            .value
        )

        self.stall_torque = (
            self.get_parameter('stall_torque')
            .value
        )

        self.encoder_ppr = (
            self.get_parameter('encoder_ppr')
            .value
        )

        # ==========================================================
        # CONVERTE 750 RPM PARA RAD/S
        # ==========================================================

        self.no_load_speed = (
            self.motor_rpm
            * 2.0
            * math.pi
            / 60.0
        )

        # ≈ 78.54 rad/s

        # ==========================================================
        # ESTADO DAS RODAS
        # ==========================================================

        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.left_position = 0.0
        self.right_position = 0.0

        # Comando solicitado pelo robô.
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0

        # ==========================================================
        # PUBLICADOR DE TORQUE
        # ==========================================================

        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            'wheel_effort_controller/commands',
            10
        )

        # ==========================================================
        # RECEBE O MESMO CMD_VEL QUE USÁVAMOS ANTES
        # ==========================================================
        #
        # Dessa forma NÃO precisamos alterar ataque_simples.py.
        #
        # carro1:
        # /carro1/diff_drive_controller/cmd_vel
        #
        # carro2:
        # /carro2/diff_drive_controller/cmd_vel
        # ==========================================================

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            'diff_drive_controller/cmd_vel',
            self.cmd_callback,
            10
        )

        # ==========================================================
        # LÊ VELOCIDADE E POSIÇÃO DAS RODAS
        # ==========================================================

        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            20
        )

        # Controle a 100 Hz.
        self.timer = self.create_timer(
            0.01,
            self.control_loop
        )

        self.get_logger().info(
            'Modelo N20 iniciado.'
        )

        self.get_logger().info(
            f'Velocidade sem carga: '
            f'{self.no_load_speed:.2f} rad/s '
            f'({self.motor_rpm:.0f} RPM)'
        )

        self.get_logger().info(
            f'Torque de stall: '
            f'{self.stall_torque:.4f} N.m'
        )

        self.get_logger().info(
            f'Encoder: '
            f'{self.encoder_ppr} PPR'
        )

    # ==============================================================
    # COMANDO DE VELOCIDADE DO ROBÔ
    # ==============================================================

    def cmd_callback(self, msg):

        self.linear_cmd = (
            msg.twist.linear.x
        )

        self.angular_cmd = (
            msg.twist.angular.z
        )

    # ==============================================================
    # LEITURA DAS JUNTAS
    # ==============================================================

    def joint_callback(self, msg):

        for i, name in enumerate(msg.name):

            if name == 'left_wheel_joint':

                if i < len(msg.velocity):
                    self.left_velocity = (
                        msg.velocity[i]
                    )

                if i < len(msg.position):
                    self.left_position = (
                        msg.position[i]
                    )

            elif name == 'right_wheel_joint':

                if i < len(msg.velocity):
                    self.right_velocity = (
                        msg.velocity[i]
                    )

                if i < len(msg.position):
                    self.right_position = (
                        msg.position[i]
                    )

    # ==============================================================
    # MODELO SIMPLIFICADO DO MOTOR DC
    # ==============================================================

    def calcular_torque(
        self,
        velocidade_desejada,
        velocidade_real
    ):

        # ----------------------------------------------------------
        # A velocidade desejada vira uma fração da tensão/PWM.
        #
        # Exemplo:
        #
        # desejado = 78.54 rad/s
        # duty = 1.0
        #
        # desejado = 39.27 rad/s
        # duty = 0.5
        # ----------------------------------------------------------

        duty = (
            velocidade_desejada
            / self.no_load_speed
        )

        # Limita entre -100% e +100%.
        duty = max(
            -1.0,
            min(1.0, duty)
        )

        # Se não existe comando, deixamos o motor em COAST.
        if abs(duty) < 0.0001:
            return 0.0

        # ----------------------------------------------------------
        # MODELO TORQUE x VELOCIDADE
        #
        # tau = tau_stall *
        #       (duty - omega / omega0)
        #
        # Para 100%:
        #
        # omega = 0
        # torque = stall
        #
        # omega = omega0
        # torque = 0
        # ----------------------------------------------------------

        torque = (
            self.stall_torque
            * (
                duty
                - velocidade_real
                / self.no_load_speed
            )
        )

        # Não deixa ultrapassar o torque de stall.
        torque = max(
            -self.stall_torque,
            min(
                self.stall_torque,
                torque
            )
        )

        return torque

    # ==============================================================
    # LOOP PRINCIPAL
    # ==============================================================

    def control_loop(self):

        # ----------------------------------------------------------
        # CINEMÁTICA DIFERENCIAL
        #
        # esquerda:
        # v - omega*L/2
        #
        # direita:
        # v + omega*L/2
        # ----------------------------------------------------------

        left_linear = (
            self.linear_cmd
            - self.angular_cmd
            * self.wheel_separation
            / 2.0
        )

        right_linear = (
            self.linear_cmd
            + self.angular_cmd
            * self.wheel_separation
            / 2.0
        )

        # Velocidade linear -> angular da roda.
        left_target = (
            left_linear
            / self.wheel_radius
        )

        right_target = (
            right_linear
            / self.wheel_radius
        )

        # Calcula torque disponível.
        left_torque = self.calcular_torque(
            left_target,
            self.left_velocity
        )

        right_torque = self.calcular_torque(
            right_target,
            self.right_velocity
        )

        # Envia os dois torques.
        msg = Float64MultiArray()

        msg.data = [
            float(left_torque),
            float(right_torque)
        ]

        self.effort_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = MotorN20()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Zera o torque ao encerrar.
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]

        node.effort_pub.publish(msg)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool


class AtaqueToF(Node):

    def __init__(self):

        super().__init__('ataque_tof')

        # =====================================================
        # PARÂMETROS
        # =====================================================

        self.declare_parameter(
            'distancia_deteccao',
            0.50
        )

        self.declare_parameter(
            'torque_ataque',
            0.009
        )

        self.distancia_deteccao = (
            self.get_parameter(
                'distancia_deteccao'
            ).value
        )

        self.torque_ataque = (
            self.get_parameter(
                'torque_ataque'
            ).value
        )

        # =====================================================
        # ESTADOS
        # =====================================================

        # False até clicar em INICIAR LUTA
        self.luta_iniciada = False

        # False até encontrar o adversário
        self.atacando = False

        self.ultima_distancia = None

        # =====================================================
        # MOTOR
        # =====================================================

        self.motor_pub = self.create_publisher(
            Float64MultiArray,
            'wheel_effort_controller/commands',
            10
        )

        # =====================================================
        # SENSOR TOF
        # =====================================================

        self.tof_sub = self.create_subscription(
            LaserScan,
            'tof/front/scan',
            self.tof_callback,
            qos_profile_sensor_data
        )

        # =====================================================
        # EVENTO: NOVA LUTA
        # =====================================================

        self.reset_sub = self.create_subscription(
            Bool,
            '/nova_luta',
            self.nova_luta_callback,
            10
        )

        # =====================================================
        # EVENTO: INICIAR LUTA
        # =====================================================

        self.inicio_sub = self.create_subscription(
            Bool,
            '/iniciar_luta',
            self.iniciar_luta_callback,
            10
        )

        # =====================================================
        # LOOP DOS MOTORES - 50 Hz
        # =====================================================

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            'Agente de ataque ToF iniciado.'
        )

        self.get_logger().info(
            'Estado: AGUARDANDO NOVA LUTA / INÍCIO'
        )

    # =========================================================
    # EVENTO NOVA LUTA
    # =========================================================

    def nova_luta_callback(self, msg):

        if not msg.data:
            return

        # Volta tudo ao estado inicial
        self.luta_iniciada = False
        self.atacando = False
        self.ultima_distancia = None

        # Segurança
        self.enviar_torque(
            0.0,
            0.0
        )

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            'NOVA LUTA RECEBIDA'
        )

        self.get_logger().info(
            'Agente resetado.'
        )

        self.get_logger().info(
            'Estado: AGUARDANDO INÍCIO'
        )

    # =========================================================
    # EVENTO INICIAR LUTA
    # =========================================================

    def iniciar_luta_callback(self, msg):

        if not msg.data:
            return

        # Garante que não carregamos o ataque anterior
        self.atacando = False
        self.ultima_distancia = None

        # Agora o ToF pode tomar decisões
        self.luta_iniciada = True

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            '>>> LUTA INICIADA <<<'
        )

        self.get_logger().info(
            'Sensores liberados.'
        )

        self.get_logger().info(
            'Procurando adversário...'
        )

    # =========================================================
    # TOF
    # =========================================================

    def tof_callback(self, msg):

        # -----------------------------------------------------
        # Antes do botão INICIAR LUTA:
        # ignorar completamente o sensor
        # -----------------------------------------------------

        if not self.luta_iniciada:
            return

        # Já entrou em ataque
        if self.atacando:
            return

        distancias_validas = []

        # Temos atualmente 5 raios no ToF
        for distancia in msg.ranges:

            # Ignora infinito e NaN
            if not math.isfinite(distancia):
                continue

            if distancia < msg.range_min:
                continue

            if distancia > msg.range_max:
                continue

            distancias_validas.append(
                distancia
            )

        # Nada encontrado
        if not distancias_validas:
            return

        # Usa o raio que encontrou o objeto mais próximo
        distancia = min(
            distancias_validas
        )

        self.ultima_distancia = distancia

        # =====================================================
        # ADVERSÁRIO DETECTADO
        # =====================================================

        if distancia <= self.distancia_deteccao:

            self.atacando = True

            self.get_logger().info(
                f'ADVERSÁRIO DETECTADO: '
                f'{distancia:.3f} m'
            )

            self.get_logger().info(
                '>>> ATAQUE <<<'
            )

    # =========================================================
    # PUBLICAR TORQUE
    # =========================================================

    def enviar_torque(
        self,
        esquerda,
        direita
    ):

        msg = Float64MultiArray()

        msg.data = [
            float(esquerda),
            float(direita)
        ]

        self.motor_pub.publish(msg)

    # =========================================================
    # LOOP DE CONTROLE
    # =========================================================

    def control_loop(self):

        # Só pode aplicar torque quando:
        #
        # 1. a luta foi iniciada
        # 2. o adversário foi detectado

        if (
            self.luta_iniciada
            and
            self.atacando
        ):

            self.enviar_torque(
                self.torque_ataque,
                self.torque_ataque
            )

        else:

            self.enviar_torque(
                0.0,
                0.0
            )


def main(args=None):

    rclpy.init(args=args)

    node = AtaqueToF()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Segurança ao fechar
        node.enviar_torque(
            0.0,
            0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

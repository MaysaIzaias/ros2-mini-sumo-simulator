import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


class AtaqueSimples(Node):
    """
    Controle extremamente simples para o robô de sumô.

    Estados:
        ESPERANDO -> espera o ToF detectar alguma coisa.
        ATACANDO  -> anda reto durante um período definido.
        PARADO    -> para depois do ataque.

    O mesmo código pode ser usado pelo carro1 e pelo carro2.
    A separação acontece através dos namespaces ROS 2.
    """

    def __init__(self):
        super().__init__('ataque_simples')

        # ==========================================================
        # PARÂMETROS
        # ==========================================================

        # Velocidade para avançar contra o adversário.
        self.declare_parameter('velocidade', 0.10)

        # Distância máxima na qual consideramos que encontramos
        # um adversário.
        #
        # Como o ToF simulado alcança até 2 m, poderíamos usar
        # um valor maior. Para o Dohyo atual, 0.50 m é suficiente.
        self.declare_parameter('distancia_deteccao', 0.50)

        # Depois que o adversário for detectado, o robô continua
        # avançando durante este tempo.
        #
        # Isso é importante porque o sensor possui uma distância
        # mínima e pode parar de medir quando estiver quase encostando.
        self.declare_parameter('tempo_ataque', 2.0)

        self.velocidade = (
            self.get_parameter('velocidade')
            .get_parameter_value()
            .double_value
        )

        self.distancia_deteccao = (
            self.get_parameter('distancia_deteccao')
            .get_parameter_value()
            .double_value
        )

        self.tempo_ataque = (
            self.get_parameter('tempo_ataque')
            .get_parameter_value()
            .double_value
        )

        # ==========================================================
        # ESTADO INTERNO
        # ==========================================================

        # O robô começa parado esperando alguma detecção.
        self.estado = 'ESPERANDO'

        # Guarda o instante em que o ataque começou.
        self.inicio_ataque = None

        # Guarda apenas para diagnóstico a última distância válida.
        self.ultima_distancia = None

        # ==========================================================
        # PUBLICADOR DE VELOCIDADE
        # ==========================================================
        #
        # Repare que NÃO colocamos "/" no começo do tópico.
        #
        # Isso permite que o namespace seja aplicado:
        #
        # namespace carro1:
        # /carro1/diff_drive_controller/cmd_vel
        #
        # namespace carro2:
        # /carro2/diff_drive_controller/cmd_vel
        # ==========================================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            'diff_drive_controller/cmd_vel',
            10
        )

        # ==========================================================
        # SENSOR TOF FRONTAL
        # ==========================================================
        #
        # Também usamos tópico relativo.
        #
        # carro1:
        # /carro1/tof/front/scan
        #
        # carro2:
        # /carro2/tof/front/scan
        # ==========================================================

        self.tof_sub = self.create_subscription(
            LaserScan,
            'tof/front/scan',
            self.tof_callback,
            10
        )

        # ==========================================================
        # LOOP DE CONTROLE
        # ==========================================================
        #
        # 0.05 s = 20 Hz.
        # ==========================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Controle de ataque iniciado.'
        )

        self.get_logger().info(
            'Estado: ESPERANDO adversário...'
        )

    # ==============================================================
    # CALLBACK DO TOF
    # ==============================================================

    def tof_callback(self, msg):

        # Ainda não queremos procurar outro adversário depois
        # que o ataque já começou.
        if self.estado != 'ESPERANDO':
            return

        # Se a mensagem não contém nenhuma leitura, não fazemos nada.
        if len(msg.ranges) == 0:
            return

        # ----------------------------------------------------------
        # Filtra somente distâncias válidas.
        # ----------------------------------------------------------
        #
        # .inf significa que nada foi encontrado.
        #
        # Também ignoramos valores fora do range configurado
        # no próprio sensor.
        # ----------------------------------------------------------

        distancias_validas = []

        for distancia in msg.ranges:

            if not math.isfinite(distancia):
                continue

            if distancia < msg.range_min:
                continue

            if distancia > msg.range_max:
                continue

            distancias_validas.append(distancia)

        # Nenhuma superfície foi encontrada.
        if len(distancias_validas) == 0:
            return

        # Como nosso ToF possui apenas um raio, normalmente teremos
        # um único valor. Mesmo assim usamos min() para deixar o
        # código robusto.
        distancia = min(distancias_validas)

        self.ultima_distancia = distancia

        # ----------------------------------------------------------
        # Verifica se está perto o suficiente.
        # ----------------------------------------------------------

        if distancia <= self.distancia_deteccao:

            self.get_logger().info(
                f'ADVERSÁRIO DETECTADO a {distancia:.3f} m!'
            )

            self.get_logger().info(
                'Iniciando ataque.'
            )

            # Guarda o instante inicial.
            self.inicio_ataque = self.get_clock().now()

            # Muda de estado.
            self.estado = 'ATACANDO'

    # ==============================================================
    # PUBLICAR VELOCIDADE
    # ==============================================================

    def enviar_velocidade(self, linear, angular=0.0):

        msg = TwistStamped()

        # Usa o relógio do ROS / Gazebo.
        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # Para frente.
        msg.twist.linear.x = float(linear)

        # Giro.
        msg.twist.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    # ==============================================================
    # LOOP PRINCIPAL
    # ==============================================================

    def control_loop(self):

        # ----------------------------------------------------------
        # ESTADO 1: ESPERANDO
        # ----------------------------------------------------------

        if self.estado == 'ESPERANDO':

            # Enquanto não encontramos ninguém, ficamos parados.
            self.enviar_velocidade(
                0.0,
                0.0
            )

            return

        # ----------------------------------------------------------
        # ESTADO 2: ATACANDO
        # ----------------------------------------------------------

        if self.estado == 'ATACANDO':

            agora = self.get_clock().now()

            tempo_decorrido = (
                agora - self.inicio_ataque
            ).nanoseconds / 1e9

            # Ainda estamos dentro do período de ataque.
            if tempo_decorrido < self.tempo_ataque:

                # Anda reto.
                self.enviar_velocidade(
                    self.velocidade,
                    0.0
                )

                return

            # O tempo acabou.
            self.enviar_velocidade(
                0.0,
                0.0
            )

            self.estado = 'PARADO'

            self.get_logger().info(
                'Ataque finalizado. Robô parado.'
            )

            return

        # ----------------------------------------------------------
        # ESTADO 3: PARADO
        # ----------------------------------------------------------

        if self.estado == 'PARADO':

            self.enviar_velocidade(
                0.0,
                0.0
            )


def main(args=None):

    rclpy.init(args=args)

    node = AtaqueSimples()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Garante que o robô pare ao fechar o programa.
        node.enviar_velocidade(
            0.0,
            0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

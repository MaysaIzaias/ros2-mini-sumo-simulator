import math
import random
import subprocess
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool


# ============================================================
# CONFIGURAÇÃO DA ARENA / ROBÔ
# ============================================================

WORLD_NAME = 'pista'

DOHYO_RADIUS = 0.385

ROBOT_LENGTH = 0.100
ROBOT_WIDTH = 0.100

# Bordas internas das Shikiri
SHIKIRI_INNER_Y = 0.050

# Altura já validada na nossa simulação
ROBOT_Z = 0.075


# ============================================================
# VALIDAÇÃO DA POSE
# ============================================================

def pose_valida(x, y, yaw, lado):

    half_length = ROBOT_LENGTH / 2.0
    half_width = ROBOT_WIDTH / 2.0

    cantos = [
        (+half_length, +half_width),
        (+half_length, -half_width),
        (-half_length, +half_width),
        (-half_length, -half_width),
    ]

    c = math.cos(yaw)
    s = math.sin(yaw)

    for lx, ly in cantos:

        wx = x + lx * c - ly * s
        wy = y + lx * s + ly * c

        # ---------------------------------------------
        # Robô inteiro precisa permanecer no Dohyo
        # ---------------------------------------------

        if math.hypot(wx, wy) > DOHYO_RADIUS:
            return False

        # ---------------------------------------------
        # Cada robô permanece na própria metade
        # atrás da Shikiri
        # ---------------------------------------------

        if lado == +1:

            if wy < SHIKIRI_INNER_Y:
                return False

        else:

            if wy > -SHIKIRI_INNER_Y:
                return False

    return True


def gerar_pose(lado):

    for _ in range(10000):

        x = random.uniform(-0.30, 0.30)

        if lado == +1:
            y = random.uniform(0.10, 0.30)

        else:
            y = random.uniform(-0.30, -0.10)

        yaw = random.uniform(-math.pi, math.pi)

        if pose_valida(x, y, yaw, lado):
            return x, y, yaw

    raise RuntimeError(
        'Não foi possível gerar uma pose inicial válida.'
    )


# ============================================================
# NÓ ROS
# ============================================================

class ControleLuta(Node):

    def __init__(self):

        super().__init__('controle_luta')

        # Torque carro 1
        self.pub_motor1 = self.create_publisher(
            Float64MultiArray,
            '/carro1/wheel_effort_controller/commands',
            10
        )

        # Torque carro 2
        self.pub_motor2 = self.create_publisher(
            Float64MultiArray,
            '/carro2/wheel_effort_controller/commands',
            10
        )

        # Evento usado posteriormente para avisar os agentes
        # de que uma nova luta começou.
        self.pub_reset = self.create_publisher(
            Bool,
            '/nova_luta',
            10
        )

        # Aviso de que a luta deve começar.
        #
        # Os dois robôs recebem este sinal ao mesmo tempo.
        self.pub_inicio = self.create_publisher(
            Bool,
            '/iniciar_luta',
            10
        )



    # ========================================================
    # AVISAR NOVA LUTA
    # ========================================================

    def avisar_nova_luta(self):

        msg = Bool()
        msg.data = True

        self.pub_reset.publish(msg)

        self.get_logger().info(
            'Nova luta preparada.'
        )


    # ========================================================
    # INICIAR LUTA
    # ========================================================

    def iniciar_luta(self):

        msg = Bool()
        msg.data = True

        self.pub_inicio.publish(msg)

        self.get_logger().info(
            '>>> LUTA INICIADA <<<'
        )
    # ========================================================
    # TORQUE ZERO
    # ========================================================

    def parar_motores(self):

        msg = Float64MultiArray()

        msg.data = [
            0.0,
            0.0
        ]

        self.pub_motor1.publish(msg)
        self.pub_motor2.publish(msg)

    # ========================================================
    # EXECUTAR SERVIÇO GAZEBO
    # ========================================================

    def executar_gz(self, argumentos):

        resultado = subprocess.run(
            argumentos,
            capture_output=True,
            text=True,
            timeout=5
        )

        if resultado.returncode != 0:

            raise RuntimeError(
                resultado.stderr.strip()
                or
                resultado.stdout.strip()
            )

        if 'true' not in resultado.stdout.lower():

            raise RuntimeError(
                'Gazebo não confirmou a operação:\n'
                + resultado.stdout
            )

    # ========================================================
    # PAUSAR / RETOMAR
    # ========================================================

    def pausar(self, valor):

        texto = 'true' if valor else 'false'

        self.executar_gz([
            'gz',
            'service',

            '-s',
            f'/world/{WORLD_NAME}/control',

            '--reqtype',
            'gz.msgs.WorldControl',

            '--reptype',
            'gz.msgs.Boolean',

            '--timeout',
            '3000',

            '--req',
            f'pause: {texto}'
        ])

    # ========================================================
    # TELEPORTAR ROBÔ
    # ========================================================

    def definir_pose(
        self,
        nome,
        x,
        y,
        yaw
    ):

        # Conversão yaw -> quaternion

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        request = (
            f'name: "{nome}", '
            f'position: {{'
            f'x: {x}, '
            f'y: {y}, '
            f'z: {ROBOT_Z}'
            f'}}, '
            f'orientation: {{'
            f'x: 0.0, '
            f'y: 0.0, '
            f'z: {qz}, '
            f'w: {qw}'
            f'}}'
        )

        self.executar_gz([
            'gz',
            'service',

            '-s',
            f'/world/{WORLD_NAME}/set_pose/blocking',

            '--reqtype',
            'gz.msgs.Pose',

            '--reptype',
            'gz.msgs.Boolean',

            '--timeout',
            '5000',

            '--req',
            request
        ])


# ============================================================
# INTERFACE
# ============================================================

class JanelaControle:

    def __init__(self, root, node):

        self.root = root
        self.node = node

        self.numero_luta = 0

        root.title('Mini Sumô - Controle de Lutas')

        # Um pouco maior para acomodar os dois botões
        root.geometry('440x390')

        root.resizable(
            False,
            False
        )

        frame = ttk.Frame(
            root,
            padding=20
        )

        frame.pack(
            fill='both',
            expand=True
        )

        # ========================================================
        # TÍTULO
        # ========================================================

        titulo = ttk.Label(
            frame,
            text='MINI SUMÔ',
            font=('Arial', 18, 'bold')
        )

        titulo.pack(
            pady=(0, 15)
        )

        # ========================================================
        # BOTÃO NOVA LUTA
        # ========================================================

        self.botao_nova = ttk.Button(
            frame,
            text='NOVA LUTA',
            command=self.nova_luta
        )

        self.botao_nova.pack(
            ipadx=30,
            ipady=12,
            pady=(10, 5)
        )

        # ========================================================
        # BOTÃO INICIAR LUTA
        # ========================================================

        self.botao_iniciar = ttk.Button(
            frame,
            text='INICIAR LUTA',
            command=self.iniciar_luta,
            state='disabled'
        )

        self.botao_iniciar.pack(
            ipadx=30,
            ipady=12,
            pady=(5, 10)
        )

        # ========================================================
        # INFORMAÇÕES DA LUTA
        # ========================================================

        self.label_luta = ttk.Label(
            frame,
            text='Luta #0',
            font=('Arial', 12, 'bold')
        )

        self.label_luta.pack(
            pady=10
        )

        self.label_carro1 = ttk.Label(
            frame,
            text='Carro 1: aguardando...'
        )

        self.label_carro1.pack(
            pady=5
        )

        self.label_carro2 = ttk.Label(
            frame,
            text='Carro 2: aguardando...'
        )

        self.label_carro2.pack(
            pady=5
        )

        self.label_status = ttk.Label(
            frame,
            text='Simulação pronta.'
        )

        self.label_status.pack(
            pady=15
        )

    # ========================================================
    # NOVA LUTA
    # ========================================================

    def nova_luta(self):

        # Impede cliques durante o reset
        self.botao_nova.config(
            state='disabled'
        )

        self.botao_iniciar.config(
            state='disabled'
        )

        self.label_status.config(
            text='Preparando nova luta...'
        )

        self.root.update_idletasks()

        pausado = False

        try:

            # ------------------------------------------------
            # 1. Parar motores
            # ------------------------------------------------

            self.node.parar_motores()

            # ------------------------------------------------
            # 2. Pausar Gazebo
            # ------------------------------------------------

            self.node.pausar(True)

            pausado = True

            # ------------------------------------------------
            # 3. Gerar poses válidas
            # ------------------------------------------------

            carro1 = gerar_pose(+1)
            carro2 = gerar_pose(-1)

            x1, y1, yaw1 = carro1
            x2, y2, yaw2 = carro2

            # ------------------------------------------------
            # 4. Reposicionar carro 1
            # ------------------------------------------------

            self.node.definir_pose(
                'carro1',
                x1,
                y1,
                yaw1
            )

            # ------------------------------------------------
            # 5. Reposicionar carro 2
            # ------------------------------------------------

            self.node.definir_pose(
                'carro2',
                x2,
                y2,
                yaw2
            )

            # ------------------------------------------------
            # 6. Garantir motores em zero
            # ------------------------------------------------

            self.node.parar_motores()

            # ------------------------------------------------
            # 7. Resetar agentes
            # ------------------------------------------------

            self.node.avisar_nova_luta()

            # ------------------------------------------------
            # 8. Retomar Gazebo
            # ------------------------------------------------

            self.node.pausar(False)

            pausado = False

            # ------------------------------------------------
            # 9. Atualizar painel
            # ------------------------------------------------

            self.numero_luta += 1

            self.label_luta.config(
                text=f'Luta #{self.numero_luta}'
            )

            self.label_carro1.config(
                text=(
                    f'Carro 1: '
                    f'x={x1:.3f}  '
                    f'y={y1:.3f}  '
                    f'θ={math.degrees(yaw1):.1f}°'
                )
            )

            self.label_carro2.config(
                text=(
                    f'Carro 2: '
                    f'x={x2:.3f}  '
                    f'y={y2:.3f}  '
                    f'θ={math.degrees(yaw2):.1f}°'
                )
            )

            # Agora sim está pronto para iniciar
            self.label_status.config(
                text='Posições prontas. Aguardando início.'
            )

            self.botao_iniciar.config(
                state='normal'
            )

        except Exception as erro:

            self.label_status.config(
                text=f'ERRO: {erro}'
            )

            if pausado:

                try:
                    self.node.pausar(False)

                except Exception:
                    pass

        finally:

            self.botao_nova.config(
                state='normal'
            )

    # ========================================================
    # INICIAR LUTA
    # ========================================================

    def iniciar_luta(self):

        # Não deixa apertar INICIAR duas vezes
        self.botao_iniciar.config(
            state='disabled'
        )

        # Publica /iniciar_luta
        self.node.iniciar_luta()

        # Atualiza interface
        self.label_status.config(
            text='LUTA EM ANDAMENTO!'
        )

# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ControleLuta()

    root = tk.Tk()

    janela = JanelaControle(
        root,
        node
    )

    try:

        root.mainloop()

    finally:

        node.parar_motores()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

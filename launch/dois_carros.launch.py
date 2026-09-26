import os
import random
import math
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import (
    get_package_share_directory,
)

# ============================================================
# POSIÇÃO INICIAL ALEATÓRIA - MINI SUMÔ
# ============================================================

DOHYO_RADIUS = 0.385

# Robô: 10 cm x 10 cm
ROBOT_LENGTH = 0.100
ROBOT_WIDTH = 0.100

# Shikiri:
#
# separação entre as bordas internas = 10 cm
#
# portanto:
#
# carro1 deve ficar completamente acima de y = +0.05
# carro2 deve ficar completamente abaixo de y = -0.05
SHIKIRI_INNER_Y = 0.050


def pose_valida(x, y, yaw, lado):
    """
    Verifica se os quatro cantos do robô:

    1. estão dentro do Dohyo;
    2. estão atrás da Shikiri correspondente.

    lado:
        +1 -> carro1
        -1 -> carro2
    """

    half_length = ROBOT_LENGTH / 2.0
    half_width = ROBOT_WIDTH / 2.0

    # Cantos do robô no sistema local
    cantos_locais = [
        (+half_length, +half_width),
        (+half_length, -half_width),
        (-half_length, +half_width),
        (-half_length, -half_width),
    ]

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for local_x, local_y in cantos_locais:

        # Rotação + translação do canto
        world_x = (
            x
            + local_x * cos_yaw
            - local_y * sin_yaw
        )

        world_y = (
            y
            + local_x * sin_yaw
            + local_y * cos_yaw
        )

        # ----------------------------------------------------
        # 1. O canto precisa estar dentro do Dohyo
        # ----------------------------------------------------

        distancia_centro = math.sqrt(
            world_x ** 2
            + world_y ** 2
        )

        if distancia_centro > DOHYO_RADIUS:
            return False

        # ----------------------------------------------------
        # 2. Precisa estar atrás da Shikiri
        # ----------------------------------------------------

        if lado == +1:

            # carro1 ocupa a metade +Y
            if world_y < SHIKIRI_INNER_Y:
                return False

        elif lado == -1:

            # carro2 ocupa a metade -Y
            if world_y > -SHIKIRI_INNER_Y:
                return False

    return True


def gerar_pose_inicial(lado):
    """
    Sorteia uma pose legal.

    Retorna:
        x, y, yaw
    """

    while True:

        # -----------------------------------------------
        # Posição aleatória
        # -----------------------------------------------

        # Deixamos uma área ampla para o sorteio.
        # A função pose_valida rejeita posições ilegais.
        x = random.uniform(
            -0.30,
            +0.30
        )

        if lado == +1:

            y = random.uniform(
                +0.10,
                +0.30
            )

        else:

            y = random.uniform(
                -0.30,
                -0.10
            )

        # -----------------------------------------------
        # Orientação completamente aleatória
        # -----------------------------------------------

        yaw = random.uniform(
            -math.pi,
            +math.pi
        )

        # -----------------------------------------------
        # Só aceita se TODA a geometria for legal
        # -----------------------------------------------

        if pose_valida(
            x,
            y,
            yaw,
            lado
        ):
            return x, y, yaw



def generate_launch_description():

    # ============================================================
    # CAMINHOS DO PACOTE
    # ============================================================

    carro_sim_share = get_package_share_directory('carro_sim')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')


    xacro_file = os.path.join(
        carro_sim_share,
        'urdf',
        'carro_multi.xacro'
    )


    world_file = os.path.join(
        carro_sim_share,
        'worlds',
        'pista_dupla.sdf'
    )


    controller_file = os.path.join(
        carro_sim_share,
        'config',
        'controllers_multi.yaml'
    )


    # ============================================================
    # ROBOT DESCRIPTION - CARRO 1
    # ============================================================

    robot1_description = ParameterValue(

        Command([

            'xacro ',
            xacro_file,

            ' robot_ns:=carro1',

            ' controller_config:=',
            controller_file,

        ]),

        value_type=str
    )


    # ============================================================
    # ROBOT DESCRIPTION - CARRO 2
    # ============================================================

    robot2_description = ParameterValue(

        Command([

            'xacro ',
            xacro_file,

            ' robot_ns:=carro2',

            ' controller_config:=',
            controller_file,

        ]),

        value_type=str
    )

    # ========================================================
    # POSES ALEATÓRIAS
    # ========================================================

    carro1_x, carro1_y, carro1_yaw = gerar_pose_inicial(+1)

    carro2_x, carro2_y, carro2_yaw = gerar_pose_inicial(-1)

    print()
    print("==========================================")
    print(" POSES INICIAIS ALEATÓRIAS")
    print("==========================================")

    print(
        f"Carro 1: "
        f"x={carro1_x:.3f}, "
        f"y={carro1_y:.3f}, "
        f"yaw={math.degrees(carro1_yaw):.1f} graus"
    )

    print(
        f"Carro 2: "
        f"x={carro2_x:.3f}, "
        f"y={carro2_y:.3f}, "
        f"yaw={math.degrees(carro2_yaw):.1f} graus"
    )

    print("==========================================")
    print()


    # ============================================================
    # GAZEBO
    # ============================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )

        ),

        launch_arguments={

            # -r = inicia a simulação em RUN
            'gz_args': '-r ' + world_file,

            # Se Gazebo fechar, encerra o launch
            'on_exit_shutdown': 'True',

        }.items()
    )


    # ============================================================
    # ROBOT STATE PUBLISHER - CARRO 1
    # ============================================================

    robot_state_publisher_1 = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        namespace='carro1',

        name='robot_state_publisher',

        output='screen',

        parameters=[

            {
                'robot_description':
                    robot1_description,

                'use_sim_time':
                    True,

                # Transforma:
                #
                # base_link
                #
                # em:
                #
                # carro1/base_link
                #
                'frame_prefix':
                    'carro1/',
            }

        ]
    )


    # ============================================================
    # ROBOT STATE PUBLISHER - CARRO 2
    # ============================================================

    robot_state_publisher_2 = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        namespace='carro2',

        name='robot_state_publisher',

        output='screen',

        parameters=[

            {
                'robot_description':
                    robot2_description,

                'use_sim_time':
                    True,

                'frame_prefix':
                    'carro2/',
            }

        ]
    )


    # ============================================================
    # SPAWN DO CARRO 1
    # ============================================================

    spawn_carro1 = Node(

        package='ros_gz_sim',

        executable='create',

        name='spawn_carro1',

        arguments=[

            '-topic',
            '/carro1/robot_description',

            '-name',
            'carro1',

            # posição inicial
            '-x',
            str(carro1_x),

            '-y',
            str(carro1_y),

            # altura adequada ao modelo atual
            '-z',
            '0.075',

            '-Y',
            str(carro1_yaw),

        ],

        output='screen'
    )


    # ============================================================
    # SPAWN DO CARRO 2
    # ============================================================

    spawn_carro2 = Node(

        package='ros_gz_sim',

        executable='create',

        name='spawn_carro2',

        arguments=[

            '-topic',
            '/carro2/robot_description',

            '-name',
            'carro2',

            '-x',
             str(carro2_x),

            '-y',
            str(carro2_y),

            '-z',
            '0.075',

            '-Y',
            str(carro2_yaw),

        ],

        output='screen'
    )


    # ============================================================
    # CONTROLLERS - CARRO 1
    # ============================================================

    joint_state_broadcaster_1 = Node(

        package='controller_manager',

        executable='spawner',

        name='spawner_jsb_carro1',

        arguments=[

            'joint_state_broadcaster',

            '--controller-manager',
            '/carro1/controller_manager',

        ],

        output='screen'
    )


    wheel_effort_controller_1 = Node(

        package='controller_manager',

        executable='spawner',

        name='spawner_effort_carro1',

        arguments=[

            'wheel_effort_controller',

            '--controller-manager',
            '/carro1/controller_manager',

        ],

        output='screen'
    )


    # ============================================================
    # CONTROLLERS - CARRO 2
    # ============================================================

    joint_state_broadcaster_2 = Node(

        package='controller_manager',

        executable='spawner',

        name='spawner_jsb_carro2',

        arguments=[

            'joint_state_broadcaster',

            '--controller-manager',
            '/carro2/controller_manager',

        ],

        output='screen'
    )


    wheel_effort_controller_2 = Node(

        package='controller_manager',

        executable='spawner',

        name='spawner_effort_carro2',

        arguments=[

            'wheel_effort_controller',

            '--controller-manager',
            '/carro2/controller_manager',

        ],

        output='screen'
    )


    # ============================================================
    # BRIDGE GAZEBO -> ROS 2
    # ============================================================
    #
    # [
    #
    # significa:
    #
    # Gazebo ---> ROS 2
    #
    # Não precisamos mandar LaserScan ROS -> Gazebo.
    # ============================================================

    bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        name='multi_robot_bridge',

        output='screen',

        arguments=[

            # ====================================================
            # CLOCK
            # ====================================================

            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[gz.msgs.Clock',


            # ====================================================
            # CARRO 1 - TOF
            # ====================================================

            '/carro1/tof/front/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',


            # ====================================================
            # CARRO 1 - SHARPS
            # ====================================================

            '/carro1/sharp/front_left/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro1/sharp/front_right/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro1/sharp/left/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro1/sharp/right/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro1/sharp/rear/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',


            # ====================================================
            # CARRO 2 - TOF
            # ====================================================

            '/carro2/tof/front/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',


            # ====================================================
            # CARRO 2 - SHARPS
            # ====================================================

            '/carro2/sharp/front_left/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro2/sharp/front_right/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro2/sharp/left/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro2/sharp/right/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/carro2/sharp/rear/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

        ]
    )


    # ============================================================
    # TF GLOBAL - CARRO 1
    # ============================================================
    #
    # world
    #   |
    #   +--- carro1/odom
    #
    # O carro nasceu em y = +0.30.
    # ============================================================

    world_to_odom_carro1 = Node(

        package='tf2_ros',

        executable='static_transform_publisher',

        name='world_to_odom_carro1',

        arguments=[

            '--x',
            str(carro1_x),

            '--y',
            str(carro1_y),

            '--z',
            '0.0',

            '--yaw',
            str(carro1_yaw),

            '--pitch',
            '0.0',

            '--roll',
            '0.0',

            '--frame-id',
            'world',

            '--child-frame-id',
            'carro1/odom',

        ],

        output='screen'
    )


    # ============================================================
    # TF GLOBAL - CARRO 2
    # ============================================================

    world_to_odom_carro2 = Node(

        package='tf2_ros',

        executable='static_transform_publisher',

        name='world_to_odom_carro2',

        arguments=[

            '--x',
            str(carro1_x),

            '--y',
            str(carro1_y),

            '--z',
            '0.0',

            '--yaw',
            str(carro1_yaw),

            '--pitch',
            '0.0',

            '--roll',
            '0.0',

            '--frame-id',
            'world',

            '--child-frame-id',
            'carro2/odom',

        ],

        output='screen'
    )


    motor_n20_carro1 = Node(

    package='carro_sim',
    executable='motor_n20',

    namespace='carro1',

    name='motor_n20',

    parameters=[{
        'use_sim_time': True,

        'wheel_radius': 0.016,
        'wheel_separation': 0.121,

        'motor_rpm': 750.0,
        'stall_torque': 0.009,

        'encoder_ppr': 140,
    }],

    output='screen'
    )

# carro 2 node
    motor_n20_carro2 = Node(

    package='carro_sim',
    executable='motor_n20',

    namespace='carro2',

    name='motor_n20',

    parameters=[{
        'use_sim_time': True,

        'wheel_radius': 0.016,
        'wheel_separation': 0.121,

        'motor_rpm': 750.0,
        'stall_torque': 0.009,

        'encoder_ppr': 140,
    }],

    output='screen'
    )

    # ============================================================
    # ORDEM DE INICIALIZAÇÃO
    # ============================================================
    #
    # Primeiro:
    #
    # spawn carro
    #
    # Depois:
    #
    # joint_state_broadcaster
    #
    # Depois:
    #
    # diff_drive_controller
    #
    # Isso evita iniciar controller antes do modelo existir.
    # ============================================================


    iniciar_jsb_carro1 = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_carro1,

            on_exit=[
                joint_state_broadcaster_1
            ]
        )
    )


    iniciar_diff_carro1 = RegisterEventHandler(

        OnProcessExit(

            target_action=joint_state_broadcaster_1,

            on_exit=[
                wheel_effort_controller_1
            ]
        )
    )


    iniciar_jsb_carro2 = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_carro2,

            on_exit=[
                joint_state_broadcaster_2
            ]
        )
 )


    iniciar_diff_carro2 = RegisterEventHandler(

        OnProcessExit(

            target_action=joint_state_broadcaster_2,

            on_exit=[
                wheel_effort_controller_2
            ]
        )
    )


    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        # Gazebo
        gazebo,

        # Bridge
        bridge,

        # Robot descriptions
        robot_state_publisher_1,
        robot_state_publisher_2,

        # TF global
        world_to_odom_carro1,
        world_to_odom_carro2,

        # Spawn
        spawn_carro1,
        spawn_carro2,

        # Eventos dos controllers
        iniciar_jsb_carro1,
        iniciar_diff_carro1,

        iniciar_jsb_carro2,
        iniciar_diff_carro2,

        motor_n20_carro2,
        motor_n20_carro1,

    ])

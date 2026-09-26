import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'carro_sim'

    pkg_path = get_package_share_directory(package_name)
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # Arquivos do projeto
    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'carro.xacro'
    )

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'pista.sdf'
    )

    controllers_file = os.path.join(
        pkg_path,
        'config',
        'controllers.yaml'
    )

    # Converte Xacro para URDF
    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_path,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Bridge para o clock do Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Insere o carro no Gazebo
    spawn_carro = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'carro',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.047'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Diff Drive Controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_file
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_carro,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_carro,
                on_exit=[
                    joint_state_broadcaster
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[
                    diff_drive_controller
                ]
            )
        ),
    ])

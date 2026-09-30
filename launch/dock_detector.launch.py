"""
Bring-up de solo detección (nodo `dock_node` + parámetros).

Para probar y depurar el detector con RViz/teleop sin depender de la máquina
de estados del controller (que sigue viviendo, sin tocar, en on_timer/controller.py).

`use_sim_time` es true por defecto (Gazebo publica /clock); en el robot real
lanzar con `use_sim_time:=false`, porque allí no hay /clock y el reloj del nodo
se quedaría en 0.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('create3_dock_solution')
    params_file = os.path.join(package_share, 'config', 'dock_detector.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true en Gazebo (reloj /clock); false en el robot real.'),
        Node(
            package='create3_dock_solution',
            executable='dock_node',
            name='dock_lidar',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])

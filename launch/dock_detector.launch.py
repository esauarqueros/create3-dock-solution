"""
Bring-up de solo detección (nodo `dock_node` + parámetros).

Para probar y depurar el detector con RViz/teleop sin depender de la máquina
de estados del controller (que sigue viviendo, sin tocar, en on_timer/controller.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('create3_dock_solution')
    params_file = os.path.join(package_share, 'config', 'dock_detector.yaml')

    return LaunchDescription([
        Node(
            package='create3_dock_solution',
            executable='dock_node',
            name='dock_lidar',
            output='screen',
            parameters=[params_file],
        ),
    ])

"""
Solución completa: nodo único `dock_node` (detector + controller + markers).

Argumentos (ver docs/LANZAMIENTO.md):
  use_sim_time      true en Gazebo (default); false en el robot real (no hay /clock).
  enable_controller false = solo detector y markers, sin publicar /cmd_vel (para teleop).
  approach_mode     point_align (default) | polar: ley para llegar al pre-dock.
  enable_markers    false = no publicar markers de RViz.
  run_report        true = guardar reporte por corrida (CSV + JSON + PNG) en report_dir.
  report_dir, tag   carpeta de salida y etiqueta libre de la corrida.

Los argumentos pisan a los YAML (config/dock_detector.yaml, config/dock_controller.yaml).
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
    config_dir = os.path.join(package_share, 'config')
    args = {
        'use_sim_time': ('true', 'true en Gazebo (reloj /clock); false en el robot real.'),
        'enable_controller': ('true', 'false: solo detector + markers, sin /cmd_vel.'),
        'approach_mode': ('point_align', 'Aproximación al pre-dock: point_align | polar.'),
        'enable_markers': ('true', 'false: no publicar markers de RViz.'),
        'run_report': ('false', 'true: guardar timeseries.csv, summary.json y run.png.'),
        'report_dir': ('/ws/dock_runs', 'Carpeta de salida del reporte.'),
        'tag': ('', 'Etiqueta libre de la corrida (aparece en el nombre de la carpeta).'),
    }

    def cfg(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in args.items()],
        Node(
            package='create3_dock_solution',
            executable='dock_node',
            name='dock_lidar',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'dock_detector.yaml'),
                os.path.join(config_dir, 'dock_controller.yaml'),
                {
                    'use_sim_time': cfg('use_sim_time', bool),
                    'controller.supervision.enable': cfg('enable_controller', bool),
                    'controller.approach.mode': cfg('approach_mode', str),
                    'debug.enable_markers': cfg('enable_markers', bool),
                    'debug.run_report.enable': cfg('run_report', bool),
                    'debug.run_report.output_dir': cfg('report_dir', str),
                    'debug.run_report.tag': cfg('tag', str),
                },
            ],
        ),
    ])

import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .detector import DockDetector
from .controller import DockController
from .types import Pose2D

class DockNode(Node):
    def __init__(self):
        super().__init__('dock_lidar')
        # declarar parámetros → dicts para detector y controller
        self.detector = DockDetector(params={})
        self.controller = DockController(params={})
        self.robot_pose = None
        self.dock = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # + /dock_status para is_docked
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.on_timer)   # 20 Hz, publicar siempre

    def on_scan(self, msg): ...   # llama a detector.update(...)
    def on_odom(self, msg): ...   # actualiza self.robot_pose (yaw desde cuaternión)
    def on_timer(self): ...       # controller.step(...) → publica Twist

def main():
    rclpy.init(); rclpy.spin(DockNode()); rclpy.shutdown()
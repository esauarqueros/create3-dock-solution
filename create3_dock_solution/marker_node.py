import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import math


class DockMarker(Node):

    def __init__(self):
        super().__init__('dock_marker')

        self.pub = self.create_publisher(
            Marker,
            '/dock_visualization',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_markers
        )

        # Pose del dock
        self.target_x = 1.0
        self.target_y = 0.2
        self.target_theta = math.radians(90.0)

        # Punto pre-dock
        self.d_pre = 0.5

        self.pre_x = (
            self.target_x -
            self.d_pre * math.cos(self.target_theta)
        )

        self.pre_y = (
            self.target_y -
            self.d_pre * math.sin(self.target_theta)
        )

    def publish_markers(self):

        # Dock final
        dock = Marker()
        dock.header.frame_id = "odom"
        dock.header.stamp = self.get_clock().now().to_msg()

        dock.ns = "dock"
        dock.id = 0
        dock.type = Marker.SPHERE
        dock.action = Marker.ADD

        dock.pose.position.x = self.target_x
        dock.pose.position.y = self.target_y
        dock.pose.position.z = 0.1

        dock.scale.x = 0.15
        dock.scale.y = 0.15
        dock.scale.z = 0.15

        dock.color.r = 1.0
        dock.color.g = 0.0
        dock.color.b = 0.0
        dock.color.a = 1.0

        self.pub.publish(dock)


        # Pre dock
        pre = Marker()
        pre.header.frame_id = "odom"
        pre.header.stamp = self.get_clock().now().to_msg()

        pre.ns = "predock"
        pre.id = 1
        pre.type = Marker.SPHERE
        pre.action = Marker.ADD

        pre.pose.position.x = self.pre_x
        pre.pose.position.y = self.pre_y
        pre.pose.position.z = 0.1

        pre.scale.x = 0.1
        pre.scale.y = 0.1
        pre.scale.z = 0.1

        pre.color.r = 0.0
        pre.color.g = 0.0
        pre.color.b = 1.0
        pre.color.a = 1.0

        self.pub.publish(pre)


def main(args=None):

    rclpy.init(args=args)

    node = DockMarker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

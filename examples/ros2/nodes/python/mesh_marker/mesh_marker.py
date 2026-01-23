#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time

class MeshMarkerNode(Node):
    def __init__(self):
        super().__init__('mesh_marker')
        self.marker_pub = self.create_publisher(Marker, "/avocado", 2)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        # Get current ROS2 time from node
        now = self.get_clock().now().to_msg()
        marker.header.stamp = now
        marker.ns = ""
        # Shape (mesh resource type - 10)
        marker.type = 10
        marker.id = 0
        marker.action = 0
        # marker.mesh_resource = "https://assets.foxglove.dev/website/blog/how-to-visualize-ros-mesh-markers/Avocado.glb"
        marker.mesh_resource = "https://raw.githubusercontent.com/gundam-global-challenge/gundam_robot/master/gundam_rx78_description/meshes/rx78_object_005-lib.dae"

        marker.mesh_use_embedded_materials = True
        # Scale
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0
        # Color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Pose
        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.marker_pub.publish(marker)
        

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
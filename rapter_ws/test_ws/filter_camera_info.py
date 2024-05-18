#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoFilter(Node):

    def __init__(self):
        super().__init__('camera_info_filter')
        
        # Create a subscriber to the /camera_info topic
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10  # QoS profile depth
        )
        
        # Create a publisher to the /camera_info2 topic
        self.publisher = self.create_publisher(
            CameraInfo,
            '/camera_info2',
            10  # QoS profile depth
        )

    def camera_info_callback(self, msg):
        # Check if the message frame_id matches the desired frame_id
        if msg.header.frame_id == "x500_depth_0/OakD-Lite/base_link/IMX214":
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_info_filter_node = CameraInfoFilter()

    rclpy.spin(camera_info_filter_node)

    camera_info_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


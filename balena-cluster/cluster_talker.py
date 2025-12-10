#!/usr/bin/env python3
"""
Cluster-aware talker that includes node identity in messages.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import os

class ClusterTalker(Node):
    def __init__(self):
        node_id = self._sanitize_node_id(os.environ.get('NODE_ID', socket.gethostname()))
        super().__init__(f'{node_id}_talker')
        
        self.node_id = node_id
        self.publisher = self.create_publisher(String, 'cluster_chat', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info(f'Cluster talker started: {self.node_id}')

    @staticmethod
    def _sanitize_node_id(name: str) -> str:
        """Sanitize node ID for ROS2 - alphanumeric/underscore, can't start with number."""
        sanitized = ''.join(c if c.isalnum() or c == '_' else '_' for c in name)
        if sanitized and sanitized[0].isdigit():
            sanitized = 'node_' + sanitized
        return sanitized or 'unnamed_node'

    def timer_callback(self):
        msg = String()
        msg.data = f'[{self.node_id}] Message #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = ClusterTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    from rclpy.executors import ExternalShutdownException
    main()

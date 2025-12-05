#!/usr/bin/env python3
"""
Cluster-aware listener that displays message source.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import os

class ClusterListener(Node):
    def __init__(self):
        node_id = os.environ.get('NODE_ID', socket.gethostname())
        super().__init__(f'{node_id}_listener')
        
        self.node_id = node_id
        self.subscription = self.create_subscription(
            String,
            'cluster_chat',
            self.listener_callback,
            10
        )
        self.seen_sources = set()
        
        self.get_logger().info(f'Cluster listener started: {self.node_id}')

    def listener_callback(self, msg):
        # Extract source from message format: [source] Message #N
        if msg.data.startswith('['):
            source = msg.data.split(']')[0][1:]
            if source not in self.seen_sources:
                self.seen_sources.add(source)
                self.get_logger().info(f'*** New source discovered: {source} (total: {len(self.seen_sources)}) ***')
        
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = ClusterListener()
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

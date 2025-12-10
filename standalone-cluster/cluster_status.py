#!/usr/bin/env python3
"""
Cluster status monitor - shows all active nodes and message rates.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import os
from datetime import datetime
from collections import defaultdict

class ClusterStatus(Node):
    def __init__(self):
        node_id = self._sanitize_node_id(os.environ.get('NODE_ID', socket.gethostname()))
        super().__init__(f'{node_id}_status')
        
        self.node_id = node_id
        self.subscription = self.create_subscription(
            String,
            'cluster_chat',
            self.message_callback,
            10
        )
        
        # Track message counts per source
        self.message_counts = defaultdict(int)
        self.last_seen = {}
        
        # Status display timer
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info(f'Cluster status monitor started: {self.node_id}')
        self.get_logger().info('Waiting for messages on /cluster_chat...')

    @staticmethod
    def _sanitize_node_id(name: str) -> str:
        """Sanitize node ID for ROS2 - alphanumeric/underscore, can't start with number."""
        sanitized = ''.join(c if c.isalnum() or c == '_' else '_' for c in name)
        if sanitized and sanitized[0].isdigit():
            sanitized = 'node_' + sanitized
        return sanitized or 'unnamed_node'

    def message_callback(self, msg):
        if msg.data.startswith('['):
            source = msg.data.split(']')[0][1:]
            self.message_counts[source] += 1
            self.last_seen[source] = datetime.now()

    def print_status(self):
        if not self.message_counts:
            self.get_logger().info('No messages received yet...')
            return
            
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'CLUSTER STATUS - {len(self.message_counts)} active talkers')
        self.get_logger().info('=' * 50)
        
        for source, count in sorted(self.message_counts.items()):
            last = self.last_seen.get(source)
            ago = (datetime.now() - last).seconds if last else '?'
            self.get_logger().info(f'  {source}: {count} msgs (last: {ago}s ago)')
        
        total = sum(self.message_counts.values())
        self.get_logger().info(f'  TOTAL: {total} messages')
        self.get_logger().info('=' * 50)

def main():
    rclpy.init()
    node = ClusterStatus()
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

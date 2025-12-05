#!/usr/bin/env python3
"""
Cluster image viewer - receives CompressedImage messages and tracks sources.
Optionally saves images to disk.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import socket
import os
from datetime import datetime
from collections import defaultdict
import cv2
import numpy as np


class ClusterImageViewer(Node):
    def __init__(self):
        node_id = os.environ.get('NODE_ID', socket.gethostname())
        super().__init__(f'{node_id}_image_viewer')
        
        self.node_id = node_id
        
        # Settings from environment
        self.save_images = os.environ.get('SAVE_IMAGES', 'false').lower() == 'true'
        self.save_path = os.environ.get('SAVE_PATH', '/shared/images')
        self.save_interval = int(os.environ.get('SAVE_INTERVAL', '10'))  # Save every N frames
        
        # Stats tracking
        self.sources = {}  # frame_id -> stats
        self.total_frames = 0
        self.total_bytes = 0
        
        # Create save directory if needed
        if self.save_images:
            os.makedirs(self.save_path, exist_ok=True)
            self.get_logger().info(f'Saving images to: {self.save_path}')
        
        # Subscribe to the cluster camera topic
        self.subscription = self.create_subscription(
            CompressedImage,
            'cluster_camera/compressed',
            self.image_callback,
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(10.0, self.print_status)
        
        self.get_logger().info(f'Cluster image viewer started: {self.node_id}')
        self.get_logger().info('Waiting for images on /cluster_camera/compressed...')

    def image_callback(self, msg: CompressedImage):
        source = msg.header.frame_id or 'unknown'
        timestamp = datetime.now()
        size_bytes = len(msg.data)
        
        # Update stats for this source
        if source not in self.sources:
            self.sources[source] = {
                'frame_count': 0,
                'total_bytes': 0,
                'first_seen': timestamp,
                'last_seen': timestamp,
            }
            self.get_logger().info(f'*** New camera source: {source} ***')
        
        stats = self.sources[source]
        stats['frame_count'] += 1
        stats['total_bytes'] += size_bytes
        stats['last_seen'] = timestamp
        
        self.total_frames += 1
        self.total_bytes += size_bytes
        
        # Optionally save image
        if self.save_images and stats['frame_count'] % self.save_interval == 0:
            self._save_image(msg, source, stats['frame_count'])
        
        # Log receipt
        size_kb = size_bytes / 1024
        self.get_logger().debug(f'Received from {source}: {size_kb:.1f} KB')

    def _save_image(self, msg: CompressedImage, source: str, frame_num: int):
        """Save a compressed image to disk."""
        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().warning(f'Failed to decode image from {source}')
                return
            
            # Generate filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            # Sanitize source name for filename
            safe_source = source.replace('/', '_').replace(' ', '_')
            filename = f'{safe_source}_{timestamp}_{frame_num:06d}.jpg'
            filepath = os.path.join(self.save_path, filename)
            
            cv2.imwrite(filepath, image)
            self.get_logger().info(f'Saved: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

    def print_status(self):
        if not self.sources:
            self.get_logger().info('No images received yet...')
            return
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'IMAGE VIEWER STATUS - {len(self.sources)} camera sources')
        self.get_logger().info('=' * 60)
        
        for source, stats in sorted(self.sources.items()):
            frames = stats['frame_count']
            size_mb = stats['total_bytes'] / (1024 * 1024)
            last_ago = (datetime.now() - stats['last_seen']).seconds
            
            # Calculate FPS
            duration = (stats['last_seen'] - stats['first_seen']).total_seconds()
            fps = frames / duration if duration > 0 else 0
            
            self.get_logger().info(
                f'  {source}: {frames} frames, {size_mb:.1f} MB, '
                f'{fps:.2f} fps, last: {last_ago}s ago'
            )
        
        total_mb = self.total_bytes / (1024 * 1024)
        self.get_logger().info(f'  TOTAL: {self.total_frames} frames, {total_mb:.1f} MB')
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    node = ClusterImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

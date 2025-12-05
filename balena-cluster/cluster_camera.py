#!/usr/bin/env python3
"""
Cluster-aware camera node that captures images from USB webcams
and publishes them as CompressedImage messages.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import socket
import os
import glob
import re
from datetime import datetime


def find_usb_webcam():
    """
    Find a USB webcam device, filtering out Pi's built-in ISP devices.
    Returns the /dev/videoN path or None if not found.
    """
    # Get list of video devices
    video_devices = sorted(glob.glob('/dev/video*'))
    
    for device in video_devices:
        # Skip high-numbered devices (Pi ISP uses video19-35)
        match = re.search(r'/dev/video(\d+)', device)
        if match:
            dev_num = int(match.group(1))
            # Pi ISP devices are typically video19+ 
            # USB webcams are usually video0, video1, etc.
            if dev_num >= 10:
                continue
        
        # Try to get device info using v4l2-ctl
        try:
            import subprocess
            result = subprocess.run(
                ['v4l2-ctl', '-d', device, '--info'],
                capture_output=True,
                text=True,
                timeout=2
            )
            info = result.stdout.lower()
            
            # Skip if it's a metadata device or ISP
            if 'meta' in info or 'pispbe' in info or 'rpi' in info:
                continue
            
            # Check if device can actually capture video
            if 'video capture' in info:
                # Try to open it with OpenCV to verify it works
                cap = cv2.VideoCapture(device)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        return device
        except Exception:
            continue
    
    return None


def find_webcam_by_index():
    """
    Fallback: Try opening video devices by index until one works.
    """
    for i in range(4):  # Try video0 through video3
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                return i
    return None


class ClusterCamera(Node):
    def __init__(self):
        node_id = os.environ.get('NODE_ID', socket.gethostname())
        super().__init__(f'{node_id}_camera')
        
        self.node_id = node_id
        self.frame_count = 0
        self.camera = None
        self.camera_device = None
        
        # Camera settings from environment
        self.capture_width = int(os.environ.get('CAMERA_WIDTH', '640'))
        self.capture_height = int(os.environ.get('CAMERA_HEIGHT', '480'))
        self.capture_fps = float(os.environ.get('CAMERA_FPS', '1.0'))  # Default 1 FPS
        self.jpeg_quality = int(os.environ.get('CAMERA_JPEG_QUALITY', '80'))
        
        # Try to find and open a webcam
        self.get_logger().info('Searching for USB webcam...')
        
        # First try the smart detection
        device = find_usb_webcam()
        if device:
            self.get_logger().info(f'Found webcam at {device}')
            self.camera = cv2.VideoCapture(device)
            self.camera_device = device
        else:
            # Fallback to index-based detection
            self.get_logger().info('Smart detection failed, trying index-based...')
            idx = find_webcam_by_index()
            if idx is not None:
                self.get_logger().info(f'Found webcam at index {idx}')
                self.camera = cv2.VideoCapture(idx)
                self.camera_device = f'/dev/video{idx}'
        
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().error('No USB webcam found! Camera node will not publish.')
            self.camera = None
            return
        
        # Configure camera
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        
        # Get actual resolution (may differ from requested)
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'Camera opened: {self.camera_device}')
        self.get_logger().info(f'Resolution: {actual_width}x{actual_height}')
        self.get_logger().info(f'Publishing at {self.capture_fps} FPS')
        
        # Create publisher for compressed images
        # Topic: /cluster_camera or /<node_id>/camera/compressed
        self.publisher = self.create_publisher(
            CompressedImage,
            'cluster_camera/compressed',
            10
        )
        
        # Also publish to a node-specific topic
        self.local_publisher = self.create_publisher(
            CompressedImage,
            f'{node_id}/camera/compressed',
            10
        )
        
        # Timer for periodic capture
        timer_period = 1.0 / self.capture_fps
        self.timer = self.create_timer(timer_period, self.capture_callback)
        
        self.get_logger().info(f'Cluster camera started: {self.node_id}')

    def capture_callback(self):
        if self.camera is None:
            return
        
        ret, frame = self.camera.read()
        if not ret or frame is None:
            self.get_logger().warning('Failed to capture frame')
            return
        
        # Encode as JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ret, jpeg_data = cv2.imencode('.jpg', frame, encode_params)
        if not ret:
            self.get_logger().warning('Failed to encode frame')
            return
        
        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.node_id}_camera'
        msg.format = 'jpeg'
        msg.data = jpeg_data.tobytes()
        
        # Publish to both topics
        self.publisher.publish(msg)
        self.local_publisher.publish(msg)
        
        self.frame_count += 1
        
        # Log periodically
        if self.frame_count % 10 == 0:
            size_kb = len(msg.data) / 1024
            self.get_logger().info(
                f'Published frame #{self.frame_count} ({size_kb:.1f} KB)'
            )

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()
            self.get_logger().info('Camera released')
        super().destroy_node()


def main():
    rclpy.init()
    node = ClusterCamera()
    
    if node.camera is None:
        node.get_logger().error('Exiting - no camera available')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

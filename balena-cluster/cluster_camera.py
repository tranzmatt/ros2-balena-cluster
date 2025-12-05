#!/usr/bin/env python3
"""
Cluster-aware camera node that captures images from USB webcams
and publishes them as CompressedImage messages.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import socket
import os
import glob
import re
from datetime import datetime


# QoS profile for camera streaming - "best effort, latest only"
# This prevents buffer buildup when no subscribers or slow subscribers
CAMERA_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Don't retry failed deliveries
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,  # Only keep the latest frame
    durability=QoSDurabilityPolicy.VOLATILE,  # Don't persist for late subscribers
)


def find_usb_webcam(logger=None):
    """
    Find a USB webcam device, filtering out Pi's built-in ISP devices.
    Returns the /dev/videoN path or None if not found.
    """
    def log(msg):
        if logger:
            logger.info(msg)
    
    # Get list of video devices
    video_devices = sorted(glob.glob('/dev/video*'))
    log(f'Found video devices: {video_devices}')
    
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
            
            log(f'Checking {device}: {info[:100]}...')
            
            # Skip if it's a metadata device or ISP
            if 'meta' in info or 'pispbe' in info or 'rpi' in info:
                log(f'Skipping {device}: appears to be ISP/meta device')
                continue
            
            # Check if device can actually capture video
            if 'video capture' not in info:
                log(f'Skipping {device}: not a video capture device')
                continue
                
            # Try to open it with OpenCV to verify it works
            # Use the device number with V4L2 backend explicitly
            cap = cv2.VideoCapture(dev_num, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    log(f'Success: {device} (index {dev_num}) is working')
                    return dev_num  # Return the index, not path
                else:
                    log(f'{device}: opened but failed to read frame')
            else:
                log(f'{device}: failed to open with OpenCV')
        except Exception as e:
            log(f'{device}: error during detection: {e}')
            continue
    
    return None


def find_webcam_by_index(logger=None):
    """
    Fallback: Try opening video devices by index until one works.
    """
    def log(msg):
        if logger:
            logger.info(msg)
    
    for i in range(10):  # Try video0 through video9
        try:
            log(f'Trying index {i}...')
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    log(f'Success: index {i} is working')
                    return i
                else:
                    log(f'Index {i}: opened but failed to read')
            else:
                log(f'Index {i}: failed to open')
        except Exception as e:
            log(f'Index {i}: error {e}')
    return None


class ClusterCamera(Node):
    def __init__(self):
        node_id = self._sanitize_node_id(os.environ.get('NODE_ID', socket.gethostname()))
        super().__init__(f'{node_id}_camera')
        
        self.node_id = node_id
        self.frame_count = 0
        self.camera = None
        self.camera_device = None

    @staticmethod
    def _sanitize_node_id(name: str) -> str:
        """Sanitize node ID for ROS2 - alphanumeric/underscore, can't start with number."""
        sanitized = ''.join(c if c.isalnum() or c == '_' else '_' for c in name)
        if sanitized and sanitized[0].isdigit():
            sanitized = 'node_' + sanitized
        return sanitized or 'unnamed_node'
        
        # Camera settings from environment (with safe parsing)
        self.capture_width = self._parse_int_env('CAMERA_WIDTH', 640)
        self.capture_height = self._parse_int_env('CAMERA_HEIGHT', 480)
        self.capture_fps = self._parse_float_env('CAMERA_FPS', 1.0)
        self.jpeg_quality = self._parse_int_env('CAMERA_JPEG_QUALITY', 80)
        
        # Allow forcing a specific device index via environment
        forced_device = self._parse_int_env('CAMERA_DEVICE', -1)
        
        # Try to find and open a webcam
        self.get_logger().info('Searching for USB webcam...')
        
        if forced_device >= 0:
            # Use forced device index
            self.get_logger().info(f'Using forced device index: {forced_device}')
            self.camera = cv2.VideoCapture(forced_device, cv2.CAP_V4L2)
            self.camera_device = f'/dev/video{forced_device}'
        else:
            # First try the smart detection
            device_idx = find_usb_webcam(self.get_logger())
            if device_idx is not None:
                self.get_logger().info(f'Found webcam at index {device_idx}')
                self.camera = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
                self.camera_device = f'/dev/video{device_idx}'
            else:
                # Fallback to index-based detection
                self.get_logger().info('Smart detection failed, trying index-based...')
                device_idx = find_webcam_by_index(self.get_logger())
                if device_idx is not None:
                    self.get_logger().info(f'Found webcam at index {device_idx}')
                    self.camera = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
                    self.camera_device = f'/dev/video{device_idx}'
        
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
        # Using BEST_EFFORT QoS - frames are dropped if no subscriber or slow subscriber
        # This prevents memory buildup during long runs
        self.publisher = self.create_publisher(
            CompressedImage,
            'cluster_camera/compressed',
            CAMERA_QOS
        )
        
        # Also publish to a node-specific topic
        self.local_publisher = self.create_publisher(
            CompressedImage,
            f'{node_id}/camera/compressed',
            CAMERA_QOS
        )
        
        # Timer for periodic capture
        timer_period = 1.0 / self.capture_fps
        self.timer = self.create_timer(timer_period, self.capture_callback)
        
        self.get_logger().info(f'Cluster camera started: {self.node_id}')

    def _parse_int_env(self, name: str, default: int) -> int:
        """Safely parse integer from environment, handling unexpanded shell syntax."""
        val = os.environ.get(name, '')
        # Handle unexpanded ${VAR:-default} syntax or empty
        if not val or val.startswith('${') or val.startswith('$'):
            return default
        try:
            return int(val)
        except ValueError:
            return default

    def _parse_float_env(self, name: str, default: float) -> float:
        """Safely parse float from environment, handling unexpanded shell syntax."""
        val = os.environ.get(name, '')
        # Handle unexpanded ${VAR:-default} syntax or empty
        if not val or val.startswith('${') or val.startswith('$'):
            return default
        try:
            return float(val)
        except ValueError:
            return default

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
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return
    
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

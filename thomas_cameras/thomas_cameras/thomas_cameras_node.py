#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus


class CameraPublisher(Node):
    def __init__(self, port, fps, camera_name):
        super().__init__('thomas_cameras_node')
        self.port = port
        self.fps = fps
        self.camera_name = camera_name

        self.compressed_publisher = self.create_publisher(CompressedImage, f'{camera_name}/image_compressed', 10)
        self.bridge = CvBridge()

        # Set up the diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID(camera_name)
        self.updater.add(f'{camera_name} Status', self.camera_diagnostic)

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.cap = cv2.VideoCapture(self.port)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera at {self.port}")
        else:
            self.get_logger().info(f"Started camera {self.camera_name} on {self.port} at {self.fps} FPS")

        # Initialize camera status
        self.camera_ok = False

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            self.camera_ok = False
        else:
            # Publish compressed image
            compressed_image_message = CompressedImage()
            compressed_image_message.header.stamp = self.get_clock().now().to_msg()
            compressed_image_message.format = "jpeg"

            # Compress the frame using JPEG format
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            compressed_image_message.data = buffer.tobytes()

            self.compressed_publisher.publish(compressed_image_message)
            self.camera_ok = True

        # Update the diagnostic status
        self.updater.update()

    def camera_diagnostic(self, stat: DiagnosticStatusWrapper):
        if self.camera_ok:
            stat.summary(DiagnosticStatus.OK, "Camera is operating correctly.")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Camera failed to capture frame.")
        return stat

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create a node with parameter declarations
    node = rclpy.create_node('thomas_cameras_node')

    # Declare parameters with default values
    node.declare_parameter('port', '/dev/video0')
    node.declare_parameter('fps', 30)
    node.declare_parameter('camera_name', 'camera1')

    # Get parameters
    port = node.get_parameter('port').get_parameter_value().string_value
    fps = node.get_parameter('fps').get_parameter_value().integer_value
    camera_name = node.get_parameter('camera_name').get_parameter_value().string_value

    camera_publisher = CameraPublisher(port, fps, camera_name)
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

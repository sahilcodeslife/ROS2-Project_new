#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class RealCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()
        colorCam = self.pipeline.create(dai.node.ColorCamera)
        colorCam.setBoardSocket(dai.CameraBoardSocket.RGB)
        colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        colorCam.setInterleaved(False)
        colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Create XLinkOut node for RGB stream
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        colorCam.video.link(xoutRgb.input)

        # Start the DepthAI device
        try:
            self.device = dai.Device(self.pipeline)
            self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize OAK camera: {str(e)}')
            rclpy.shutdown()
            return

        # Create ROS publisher
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.frame_count = 0

        self.get_logger().info('Real camera publisher started with OAK camera, publishing at 10 Hz, resized to 240x320.')

    def timer_callback(self):
        # Get RGB frame from OAK camera
        inRgb = self.qRgb.tryGet()
        if inRgb is not None:
            cv_img = inRgb.getCvFrame()
            # Resize to 240x320 (width x height)
            cv_img = cv2.resize(cv_img, (240, 320))
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.pub.publish(msg)
            self.frame_count += 1
            self.get_logger().info(f'Published frame {self.frame_count}')
        else:
            self.get_logger().warn('No frame received from OAK camera')

    def destroy_node(self):
        # Clean up DepthAI device
        if hasattr(self, 'device'):
            self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealCameraPublisher()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
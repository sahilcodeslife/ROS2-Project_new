#!/usr/bin/env python3

import os
import glob
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RandomCameraPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_publisher')

        # parameter for directory of images
        self.declare_parameter('image_dir', '/home/rosdev/ros2_ws/src/camera/camera/fake_images')
        img_dir = self.get_parameter('image_dir').get_parameter_value().string_value

        # collect all image files
        patterns = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        self.files = sorted(
            sum((glob.glob(os.path.join(img_dir, p)) for p in patterns), [])
        )
        if not self.files:
            self.get_logger().error(f'No images found in "{img_dir}"')
            rclpy.shutdown()
            return

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.index = 0

        self.get_logger().info(
            f'Fake camera publisher started, publishing {len(self.files)} images from "{img_dir}" at 10 Hz, resized to 240×320.'
        )
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # load next image
        path = self.files[self.index]
        cv_img = cv2.imread(path, cv2.IMREAD_COLOR)
        if cv_img is None:
            self.get_logger().warn(f'Failed to load "{path}", skipping')
        else:
            # BGR → RGB
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            # resize to 240×320 (width×height)
            cv_img = cv2.resize(cv_img, (240, 320))
            # convert and publish
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.pub.publish(msg)
            self.get_logger().info(
                f'Published frame {self.index+1}/{len(self.files)}'
            )

        # advance (and wrap)
        self.index = (self.index + 1) % len(self.files)


def main(args=None):
    rclpy.init(args=args)
    node = RandomCameraPublisher()
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

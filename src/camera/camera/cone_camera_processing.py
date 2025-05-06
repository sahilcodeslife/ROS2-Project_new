#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

        # hard‑coded path to your YOLO model
        model_path = '/home/rosdev/ros2_ws/src/camera/camera/model/my_model.pt'
        if not os.path.isfile(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.model = YOLO(model_path)

        # Determine the class ID for "cone"
        # Assumes the model has a class named 'cone'
        self.cone_class_id = None
        for idx, name in self.model.names.items():
            if name.lower() == 'cone':
                self.cone_class_id = idx
                break
        if self.cone_class_id is None:
            self.get_logger().warn("'cone' class not found in model names; defaulting to class 0")
            self.cone_class_id = 0

        # Directory to save detected-cone frames
        self.save_dir = '/home/rosdev/ros2_ws/src/camera/camera/cone_saved_frames'
        os.makedirs(self.save_dir, exist_ok=True)

        # bridge, subscriber, and processed-image publisher
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            10
        )

        self.get_logger().info('Camera processor started, waiting for images...')

    def if_cone_detected(self, image, cls_ids):
        """
        Check if any detection matches the cone class, and if so, save the frame.
        """
        if self.cone_class_id in cls_ids:
            # Build filename with timestamp
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = f'cone_{timestamp}.png'
            filepath = os.path.join(self.save_dir, filename)
            # Save BGR image
            cv2.imwrite(filepath, image)
            self.get_logger().info(f'Cone detected! Frame saved to {filepath}')

    def callback(self, msg: Image):
        # convert ROS Image → OpenCV BGR
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # run YOLO inference
        results = self.model(cv_img)[0]
        boxes = results.boxes.xyxy.cpu().numpy().astype(int)
        scores = results.boxes.conf.cpu().numpy()
        cls_ids = results.boxes.cls.cpu().numpy().astype(int)

        # draw detections on image
        for (x1, y1, x2, y2), score, cls in zip(boxes, scores, cls_ids):
            label = f"{self.model.names[cls]}:{score:.2f}"
            # box
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # label background
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(cv_img, (x1, y1 - h - 4), (x1 + w, y1), (0, 255, 0), cv2.FILLED)
            # label text
            cv2.putText(cv_img, label, (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # check and save if cone detected
        self.if_cone_detected(cv_img, cls_ids)

        # convert back to ROS Image (keep BGR encoding for display tools)
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out_msg.header = msg.header  # preserve timestamp/frame_id
        self.pub.publish(out_msg)
        self.get_logger().info(f'Published processed image with {len(boxes)} detections')


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
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

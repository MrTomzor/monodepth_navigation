#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # Добавили импорт!
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from monodepth.camera_processor import CameraProcessor


class ImageUndistorter(Node):
    def __init__(self):
        super().__init__('image_undistorter')

        # Объявление параметров
        self.declare_parameter('input_topic', 'rgb/image_raw')
        self.declare_parameter('input_camera_info_topic', 'rgb/camera_info')
        self.declare_parameter('output_topic', 'rgb/image_rectified')
        self.declare_parameter('distortion_coeffs', [-0.03, 0.0, 0.0, 0.0, 0.0])


        self.input_topic_name = self.get_parameter('input_topic').value
        self.input_cam_info_name = self.get_parameter('input_camera_info_topic').value
        self.output_topic_name = self.get_parameter('output_topic').value
        self.dist_coeffs = np.array(self.get_parameter('distortion_coeffs').value)

        self.bridge = CvBridge()
        self.camera = CameraProcessor(self.bridge, logger=self.get_logger())
        self.map1, self.map2, self.roi = None, None, None


        self.publisher = self.create_publisher(Image, self.output_topic_name, 1)

        self.create_subscription(Image, self.input_topic_name, self.image_callback, 1)
        self.create_subscription(CameraInfo, self.input_cam_info_name, self.camera_info_callback, 1)

        self.get_logger().info(f'Undistorter started. Listening to {self.input_topic_name}')

    def camera_info_callback(self, cam_info_msg):
        self.camera.set_camera_k_info(cam_info_msg)

    def image_callback(self, image_msg):
        self.camera.update_image(image_msg)

        if self.camera.k_matrix is None or self.camera.image is None:
            self.get_logger().warn("Camera intrinsic matrix not initialized yet.")
            return

        cv_image = self.camera.image
        h, w = cv_image.shape[:2]

        if self.map1 is None:

            new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera.k_matrix, self.dist_coeffs, (w, h), 1
            )
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera.k_matrix, self.dist_coeffs, None, new_camera_mtx, (w, h), cv2.CV_32FC1
            )
            self.get_logger().info("Undistort maps initialized.")


        rectified_img = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)


        x, y, w_r, h_r = self.roi
        final_img = rectified_img[y:y + h_r, x:x + w_r]


        output_msg = self.bridge.cv2_to_imgmsg(final_img, encoding='bgr8')
        output_msg.header = image_msg.header
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node_instance = ImageUndistorter()

    executor = MultiThreadedExecutor()
    executor.add_node(node_instance)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node_instance.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import tf2_ros
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from collections import deque
from scipy.interpolate import griddata

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from monodepth.midas_extension import MidasExtension
from monodepth.pointcloud_processor import PointCloudProcessor
from monodepth.scale_estimator import *
from monodepth.camera_processor import CameraProcessor


class MonocularDepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('monodepth_estimator')

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self.init_params()
        self.init_tf()
        self.init_state()
        self.init_publishers()
        self.init_subscribers()
        self.init_timer()

        self.get_logger().info("Node Started (MiDaS Depth Only Mode)")

    def init_state(self):
        self.midas = MidasExtension(model_type="DPT_Large")
        self.camera = CameraProcessor(self.bridge, logger=self.get_logger())


        self.scale = 1
        self.cloud_buffer = deque(maxlen=30)
        self.pointcloud = PointCloudProcessor(self.tf_buffer)

    def init_params(self):
        self.declare_parameter("output_depth_map_topic", "/midas/depth_view")
        self.declare_parameter("input_img_topic", "/uav1/rgb/image_raw")
        self.declare_parameter("input_camera_info_topic", "/uav1/rgb/camera_info")
        self.declare_parameter("camera_frame", "uav1/rgb")


        self.output_depth_map_topic_name = self.get_parameter("output_depth_map_topic").value
        self.input_img_topic_name = self.get_parameter("input_img_topic").value
        self.input_rgbd_color_cam_info_topic_name = self.get_parameter("input_camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value




        # self.declare_parameter("output_mask_vis_topic", "/midas/mask_visualization")
        # # Propeller mask coordinates (assuming 640x480 resolution).
        # # Format: [x, y, width, height] for each rectangle sequentially.
        # # - Left prop:  [0, 170, 220, 100] (starts at left edge x=0)
        # # - Right prop: [440, 170, 220, 100] (ends at right edge x=640)
        # self.declare_parameter("mask_rectangles", [0, 100, 250, 80, 440, 100, 250, 80])
        # self.output_mask_vis_topic = self.get_parameter("output_mask_vis_topic").value
        # self.mask_rects = self.get_parameter("mask_rectangles").value




        self.declare_parameter("output_scaled_depth_map_topic_map", "/midas/scaled_depth_view_map")
        # self.declare_parameter("output_scaled_depth_map_topic_value", "/midas/scaled_depth_view_value")
        self.declare_parameter("output_pointcloud_topic_map", "/midas/pointcloud_by_map")
        # self.declare_parameter("output_pointcloud_topic_value", "/midas/pointcloud_by_value")
        # self.declare_parameter("input_pointcloud_topic", "/uav1/ov_msckf/points_slam")
        self.declare_parameter("input_pointcloud_topic", "/uav1/lidar/points")
        self.output_scaled_depth_map_topic_name_map = self.get_parameter("output_scaled_depth_map_topic_map").value
        # self.output_scaled_depth_map_topic_name_value = self.get_parameter("output_scaled_depth_map_topic_value").value
        self.output_pointcloud_topic_map_name = self.get_parameter("output_pointcloud_topic_map").value
        # self.output_pointcloud_topic_value_name = self.get_parameter("output_pointcloud_topic_value").value
        self.input_pointclouds_topic_name = self.get_parameter("input_pointcloud_topic").value

    def init_tf(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def init_publishers(self):
        qos = 1
        self.pub_depth = self.create_publisher(Image, self.output_depth_map_topic_name, qos)
        self.pub_scaled_depth_map = self.create_publisher(Image, self.output_scaled_depth_map_topic_name_map, qos)
        # self.pub_scaled_depth_value = self.create_publisher(Image, self.output_scaled_depth_map_topic_name_value, qos)
        self.pub_pointcloud_map = self.create_publisher(PointCloud2, self.output_pointcloud_topic_map_name, qos)
        # self.pub_pointcloud_value = self.create_publisher(PointCloud2, self.output_pointcloud_topic_value_name, qos)
        # self.pub_mask_vis = self.create_publisher(Image, self.output_mask_vis_topic, qos)

    def init_subscribers(self):
        qos = 1
        self.create_subscription(
            Image, self.input_img_topic_name, self.image_callback, qos,
            callback_group=self.subscriber_cb_group
        )
        self.create_subscription(
            CameraInfo, self.input_rgbd_color_cam_info_topic_name, self.camera_info_callback, qos,
            callback_group=self.subscriber_cb_group
        )

        self.create_subscription(
            PointCloud2, self.input_pointclouds_topic_name, self.pointcloud_callback, qos,
            callback_group=self.subscriber_cb_group
        )
    def init_timer(self):
        self.timer = self.create_timer(
            0.01, self.compute_midas_pointcloud,
            callback_group=self.timer_cb_group
        )

    def image_callback(self, image_msg):
        self.camera.update_image(image_msg)

    def camera_info_callback(self, cam_info_msg):
        self.camera.set_camera_k_info(cam_info_msg)

    def pointcloud_callback(self, cloud_msg):
        if self.camera.k_matrix is None:
            self.get_logger().warn("Camera intrinsic matrix not initialized yet.")
            return

        raw_pointlcoud = self.pointcloud.read_pointcloud(cloud_msg)
        transformed_pointcloud = self.pointcloud.change_points_frame(
            self.camera_frame, cloud_msg.header.frame_id,
            self.camera_frame, raw_pointlcoud,
            cloud_msg.header.stamp
        )
        if transformed_pointcloud is None or len(transformed_pointcloud) == 0:
            self.get_logger().warn("There is no pointcloud in frame")
            return

        pointcloud_2d = self.pointcloud.project_points_3d_to_2d(transformed_pointcloud, self.camera.k_matrix)
        self.cloud_buffer.append((cloud_msg.header.stamp, pointcloud_2d))

    def compute_midas_pointcloud(self):
        current_image = self.camera.image
        current_time = self.camera.image_time

        if current_image is None:
            self.get_logger().warn("Waiting for Image", throttle_duration_sec=1.0)
            return

        if not self.cloud_buffer:
            self.get_logger().warn(f"Waiting for {self.input_pointclouds_topic_name} pointcloud data",
                                   throttle_duration_sec=1.0)
            return

        depth_color, raw_depth = self.midas.run(current_image)

        self.camera.publish_image(depth_color, self.pub_depth)

        depth_map = 1.0 / (raw_depth + 1e-12)
        current_pointcloud = self.find_pointcloud(current_time)
        scale_map = get_scale_map(current_pointcloud, depth_map)
        scaled_depth_map_by_map = depth_map * scale_map

        # masked_scaled_depth_map_by_map = self.put_mask_on_depth_map(scaled_depth_map_by_map, current_image.copy())
        # cloud_msg_map = self.pointcloud.create_cloud_msg(
        #     masked_scaled_depth_map_by_map, current_time, self.camera.k_matrix, self.camera_frame
        # )

        cloud_msg_map = self.pointcloud.create_cloud_msg(
            scaled_depth_map_by_map, current_time, self.camera.k_matrix, self.camera_frame
        )


        self.pointcloud.publish_pointcloud(self.pub_pointcloud_map, cloud_msg_map)
        inverse_depth_map_by_map = 1.0 / (scaled_depth_map_by_map + 1e-9)
        colorized_depth_image_map = self.colorize_inverse_depth(inverse_depth_map_by_map)
        self.camera.publish_image(colorized_depth_image_map, self.pub_scaled_depth_map)


    def put_mask_on_depth_map(self, depth_map, current_image):
        for i in range(0, len(self.mask_rects), 4):
            if i + 3 < len(self.mask_rects):
                x, y, w, h = self.mask_rects[i:i + 4]
                x_end = min(x + w, current_image.shape[1])
                y_end = min(y + h, current_image.shape[0])

                depth_map[y:y_end, x:x_end] = np.nan
                cv2.rectangle(current_image, (x, y), (x_end, y_end), (0, 0, 0), -1)
        self.camera.publish_image(current_image, self.pub_mask_vis)
        return depth_map

    def msg_time_to_sec(self, msg_stamp):
        return msg_stamp.sec + msg_stamp.nanosec * 1e-9

    def find_pointcloud(self, image_time):
        if not self.cloud_buffer:
            self.get_logger().warn("No pointcloud in cloud_buffer")
            return None

        img_time_sec = self.msg_time_to_sec(image_time)

        closest = min(
            self.cloud_buffer,
            key=lambda x: abs(self.msg_time_to_sec(x[0]) - img_time_sec)
        )

        self.get_logger().info(f"nejstarsi {self.msg_time_to_sec(self.cloud_buffer[0][0])}")
        self.get_logger().info(f"nejnovejsi {self.msg_time_to_sec(self.cloud_buffer[-1][0])}")
        self.get_logger().info(str(img_time_sec))

        _, pointcloud = closest
        return pointcloud

    def colorize_inverse_depth(self, inverse_depth_map):
        inverse_depth_map = np.nan_to_num(inverse_depth_map, nan=0.0, posinf=0.0, neginf=0.0)
        normalized = cv2.normalize(inverse_depth_map, None, 0, 255, cv2.NORM_MINMAX)
        normalized = normalized.astype(np.uint8)
        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_MAGMA)
        return colorized


def main(args=None):
    rclpy.init(args=args)
    node_instance = MonocularDepthEstimatorNode()

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
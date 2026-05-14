#!/usr/bin/env python3
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
        # self.midas = MidasExtension(model_type="DPT_Hybrid")
        # self.midas = MidasExtension(model_type="MiDaS_small")


        self.camera = CameraProcessor(self.bridge, logger=self.get_logger())
        self.cloud_buffer = deque(maxlen=30)
        self.pointcloud = PointCloudProcessor(self.tf_buffer)
        self.last_scale_value = 1
        self.last_scale_map = None

    def init_params(self):
        self.declare_parameter("output_depth_map_topic", "/midas/depth_view")
        self.declare_parameter("input_img_topic", "rgb/image_raw")
        self.declare_parameter("input_camera_info_topic", "rgb/camera_info")
        self.declare_parameter("camera_frame", "rgb")
        self.declare_parameter("world_frame", "local_origin")


        self.output_depth_map_topic_name = self.get_parameter("output_depth_map_topic").value
        self.input_img_topic_name = self.get_parameter("input_img_topic").value
        self.input_rgbd_color_cam_info_topic_name = self.get_parameter("input_camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.world_frame = self.get_parameter("world_frame").value

        self.declare_parameter("mask_rectangles", [0, 100, 250, 80, 400, 100, 400, 80])
        self.mask_rects = self.get_parameter("mask_rectangles").value

        self.declare_parameter("output_scaled_depth_map_topic_map", "/midas/scaled_depth_view_map")
        self.declare_parameter("output_pointcloud_topic_map", "/midas/pointcloud_by_map")
        self.declare_parameter("output_pointcloud_topic_value", "/midas/pointcloud_by_value")
        self.declare_parameter("input_pointcloud_topic", "/open_vins/points_slam")

        self.output_scaled_depth_map_topic_name_map = self.get_parameter("output_scaled_depth_map_topic_map").value
        self.output_pointcloud_topic_map_name = self.get_parameter("output_pointcloud_topic_map").value
        self.output_pointcloud_topic_value_name = self.get_parameter("output_pointcloud_topic_value").value
        self.input_pointclouds_topic_name = self.get_parameter("input_pointcloud_topic").value

    def init_tf(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def init_publishers(self):
        qos = 1
        self.pub_depth = self.create_publisher(Image, self.output_depth_map_topic_name, qos)
        self.pub_scaled_depth_map = self.create_publisher(Image, self.output_scaled_depth_map_topic_name_map, qos)
        self.pub_raw_depth_matrix = self.create_publisher(Image, '/midas/raw_depth_matrix', 1)
        self.pub_raw_depth_scalar = self.create_publisher(Image, '/midas/raw_depth_scalar', 1)
        self.pub_pointcloud_map = self.create_publisher(PointCloud2, self.output_pointcloud_topic_map_name, qos)
        self.pub_pointcloud_value = self.create_publisher(PointCloud2, self.output_pointcloud_topic_value_name, qos)

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
        self.timer = self.create_timer(0.1, self.compute_midas_pointcloud, callback_group=self.timer_cb_group)

    def image_callback(self, image_msg):
        self.camera.update_image(image_msg)

    def camera_info_callback(self, cam_info_msg):
        self.camera.set_camera_k_info(cam_info_msg)

    def pointcloud_callback(self, cloud_msg):
        if self.camera.k_matrix is None:
            self.get_logger().info("Camera intrinsic matrix not initialized yet.")
            return

        raw_pointlcoud = self.pointcloud.read_pointcloud(cloud_msg)
        if not raw_pointlcoud:
            return

        self.cloud_buffer.append((
            cloud_msg.header.stamp,  # source_time
            cloud_msg.header.frame_id,  # source_frame
            raw_pointlcoud
        ))


    def compute_midas_pointcloud(self):

        current_image = self.camera.image
        current_time = self.camera.image_time

        if current_image is None:
            self.get_logger().info("Waiting for Image")
            return

        if not self.cloud_buffer:
            self.get_logger().info(f"Waiting for {self.input_pointclouds_topic_name} pointcloud data")
            return

        depth_color, raw_depth = self.midas.run(current_image)
        self.camera.publish_image(depth_color, self.pub_depth)
        depth_map = 1.0 / (raw_depth + 1e-12)

        cloud_stamp, cloud_source_frame, raw_points = self.find_pointcloud(current_time)
        transformed = self.pointcloud.change_points_frame(
            fixed_frame=self.world_frame,
            source_frame=cloud_source_frame,
            target_frame=self.camera_frame,
            points=raw_points,
            source_time=cloud_stamp,
            target_time=current_time
        )

        if not transformed:
            self.get_logger().warn("Transform failed, skipping frame")
            return

        current_pointcloud = self.pointcloud.project_points_3d_to_2d(transformed, self.camera.k_matrix)

        # current_pointcloud = self.find_pointcloud(current_time)

        scale_map = get_scale_map(current_pointcloud, depth_map, self.last_scale_map)
        self.last_scale_map = scale_map
        scaled_depth_map_by_map = depth_map * scale_map

        # scale_value = get_scale_value(current_pointcloud, depth_map, self.last_scale_value)
        # self.last_scale_value = scale_value
        # scaled_depth_map_by_value = depth_map * scale_value

        masked_scaled_depth_map_by_map = self.put_mask_on_depth_map(scaled_depth_map_by_map, current_image.copy())
        cloud_msg_map = self.pointcloud.create_cloud_msg(
            masked_scaled_depth_map_by_map, current_time, self.camera.k_matrix, self.camera_frame
        )

        # masked_scaled_depth_map_by_value = self.put_mask_on_depth_map(scaled_depth_map_by_value, current_image.copy())
        # cloud_msg_value = self.pointcloud.create_cloud_msg(
        #     masked_scaled_depth_map_by_value, current_time, self.camera.k_matrix, self.camera_frame
        # )

        self.pointcloud.publish_pointcloud(self.pub_pointcloud_map, cloud_msg_map)
        # self.pointcloud.publish_pointcloud(self.pub_pointcloud_value, cloud_msg_value)


        # raw_matrix_msg = self.bridge.cv2_to_imgmsg(
        #     masked_scaled_depth_map_by_map.astype(np.float32), encoding='32FC1'
        # )
        # raw_matrix_msg.header.stamp = current_time
        # self.pub_raw_depth_matrix.publish(raw_matrix_msg)
        #
        # raw_scalar_msg = self.bridge.cv2_to_imgmsg(
        #     masked_scaled_depth_map_by_value.astype(np.float32), encoding='32FC1'
        # )
        # raw_scalar_msg.header.stamp = current_time
        # self.pub_raw_depth_scalar.publish(raw_scalar_msg)

        # inverse_depth_map_by_map = 1.0 / (scaled_depth_map_by_map + 1e-9)
        # colorized_depth_image_map = self.colorize_inverse_depth(inverse_depth_map_by_map)
        # self.camera.publish_image(colorized_depth_image_map, self.pub_scaled_depth_map)



    def put_mask_on_depth_map(self, depth_map, current_image):
        for i in range(0, len(self.mask_rects), 4):
            if i + 3 < len(self.mask_rects):
                x, y, w, h = self.mask_rects[i:i + 4]
                x_end = min(x + w, current_image.shape[1])
                y_end = min(y + h, current_image.shape[0])

                depth_map[y:y_end, x:x_end] = np.nan
                cv2.rectangle(current_image, (x, y), (x_end, y_end), (0, 0, 0), -1)
        return depth_map

    def msg_time_to_sec(self, msg_stamp):
        return msg_stamp.sec + msg_stamp.nanosec * 1e-9

    def find_pointcloud(self, image_time):
        if not self.cloud_buffer:
            self.get_logger().info("No pointcloud in cloud_buffer")
            return None, None, None

        img_time_sec = self.msg_time_to_sec(image_time)
        closest = min(
            self.cloud_buffer,
            key=lambda x: abs(self.msg_time_to_sec(x[0]) - img_time_sec)
        )
        return closest

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
import rclpy
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class PointCloudProcessor:
    def __init__(self, tf_buffer: tf2_ros.Buffer):
        self.tf_buffer = tf_buffer

    def read_pointcloud(self, msg):
        return list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    def change_points_frame(self, fixed_frame, source_frame, target_frame, points, time):
        try:
            tf_msg = self.tf_buffer.lookup_transform_full(
                target_frame=target_frame,
                target_time=time,
                source_frame=source_frame,
                source_time=time,
                fixed_frame=fixed_frame,
                timeout=rclpy.duration.Duration(seconds=0.8)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            print(f"WARNING: Pointcloud TF lookup transform failed: {e}")
            return []

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.eye(4)
        T[0:3, 0:3] = rot_matrix
        T[0:3, 3] = [t.x, t.y, t.z]

        transformed = []
        for x, y, z in points:
            vec = np.array([x, y, z, 1.0])
            x2, y2, z2, _ = T.dot(vec)
            if z2 > 0:
                transformed.append((x2, y2, z2))

        return transformed

    def project_points_3d_to_2d(self, pointclouds, k_matrix):
        projected_pointcloud = []
        for x, y, z in pointclouds:
            if z <= 0:
                continue

            u = ((k_matrix[0, 0] * x) / z) + k_matrix[0, 2]
            v = ((k_matrix[1, 1] * y) / z) + k_matrix[1, 2]

            projected_pointcloud.append((int(round(u)), int(round(v)), z))
        return projected_pointcloud

    def draw_2d_pointcloud(self, image, pointcloud_2d):
        for (u, v, d) in pointcloud_2d:
            u = int(u)
            v = int(v)
            d = int(d)
            cv2.circle(image, (u, v), 10, (0, 255, 0), -1)
        return image

    def create_cloud_msg(self, input_frame, time, k_matrix, output_frame):
        height, width = input_frame.shape
        points = []
        k_matrix_inv = np.linalg.inv(k_matrix)
        for v in range(0, height, 32):
            for u in range(0, width, 32):
                v_end = min(v + 32, height)
                u_end = min(u + 32, width)
                block = input_frame[v:v_end, u:u_end]
                if np.isnan(block).all():
                    continue
                z = np.nanmin(block)
                if np.isfinite(z):
                    pixel = np.array([u + (u_end - u) // 2, v + (v_end - v) // 2, 1.0])
                    ray = np.dot(k_matrix_inv, pixel)
                    x = ray[0] * z
                    y = ray[1] * z
                    z = ray[2] * z

                    r, g, b = 0, 0, 255
                    rgb = (r << 16) | (g << 8) | b
                    points.append([x, y, z, rgb])

        header = Header()
        header.stamp = time
        header.frame_id = output_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        return pc2.create_cloud(header, fields, points)

    def publish_pointcloud(self, publisher, cloud_msg):
        publisher.publish(cloud_msg)
#!/usr/bin/env python3

# ROS core
import rospy

# ROS messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

# ROS utilities
import sensor_msgs.point_cloud2 as pc2

# CV & math
import numpy as np
from cv_bridge import CvBridge


class InfraPointcloudBuilder:
    def __init__(self):
        rospy.init_node('infra_pointcloud_builder', anonymous=True)

        rospy.Subscriber('/uav1/rgbd/aligned_depth_to_color/image_raw', Image, self.callback)
        rospy.Subscriber('/uav1/rgbd/color/camera_info', CameraInfo, self.camera_info_callback)

        self.pub_pointcloud = rospy.Publisher('/infra/pointcloud', PointCloud2, queue_size=1)

        rospy.loginfo("Node Started")

        self.camera_K = None
        self.bridge = CvBridge()


    def callback(self, msg):
        if self.camera_K is None:
            rospy.logwarn("Camera intrinsics not received yet!")
            return
        rospy.loginfo("Depth image received")

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if depth_image.dtype == np.uint16:
            depth_meters = depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype == np.float32:
            depth_meters = depth_image
        else:
            rospy.logerr("Unsupported depth image format")
            return

        cloud = self.create_pointcloud(depth_meters, self.camera_K)
        #cloud.header.stamp = msg.header.stamp
        self.pub_pointcloud.publish(cloud)
        rospy.loginfo("Pointcloud published")




    def camera_info_callback(self, msg):
        if self.camera_K is None:
            self.camera_K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo(f"Received camera intrinsics:\n{self.camera_K}")

   

    def create_pointcloud(self, depth, K):
        height, width = depth.shape
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        step = 32

        points = []
        for v in range(0, height, step):
            for u in range(0, width, step):
                z = depth[v, u]
                if np.isfinite(z) and z > 0.1:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    r, g, b = 0, 0, 255
                    rgb = (r << 16) | (g << 8) | b

                    points.append([x, y, z, rgb])

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "uav1/rgbd/color_optical"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        return pc2.create_cloud(header, fields, points)


if __name__ == '__main__':
    InfraPointcloudBuilder()
    rospy.spin()

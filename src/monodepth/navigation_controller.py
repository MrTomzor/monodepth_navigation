#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import VelocityReferenceStamped
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
from monodepth_navigation.pointcloud_processor import PointCloudProcessor
from collections import deque
from mrs_msgs.srv import Vec4, Vec4Request
import tf.transformations as tft
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import Empty


class NavigationControllerNode:
    def __init__(self):
        rospy.init_node('navigation_controller', anonymous=True)
        self.init_params()
        self.init_tf()
        self.init_state()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()
        rospy.loginfo("Node Started")
        self.goto_srv = rospy.ServiceProxy("/uav1/octomap_planner/goto", Vec4)
        self.goto_srv.wait_for_service()

    def init_params(self):
        # self.safe_distance = rospy.get_param("safe_distance", 3.0)
        self.camera_frame = rospy.get_param("target_frame", "uav1/fcu_untilted")
        self.input_pointcloud_topic = rospy.get_param("input_pointcloud_topic", "/midas/pointcloud")
        self.output_velocity_topic = rospy.get_param("output_velocity_topic",
                                                     "/uav1/control_manager/velocity_reference")
        self.x_octogoal = rospy.get_param("x_octogoal", 0.0)
        self.y_octogoal = rospy.get_param("y_octogoal", 0.0)
        self.z_octogoal = rospy.get_param("z_octogoal", 0.0)
        self.yaw_octogoal = rospy.get_param("yaw_octogoal", 0.0)
        self.world_frame = "uav1/local_origin"
        self.body_frame = "uav1/fcu_untilted"

    def init_tf(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def init_state(self):
        self.drone_size = 0.02
        self.near_threshold = 3.0
        self.far_threshold = 12.0

        self.still_eps = 0.05
        self.last_pos = None

        self.traveled_path = deque(maxlen=10)

        self.pointcloud = PointCloudProcessor(self.tf_buffer)
        self.latest_pointcloud = None

    def init_publishers(self):
        self.pub_velocity = rospy.Publisher(self.output_velocity_topic, VelocityReferenceStamped, queue_size=10)

    def init_subscribers(self):
        rospy.Subscriber(self.input_pointcloud_topic, PointCloud2, self.pointcloud_callback)

    def init_timers(self):
        # self.timer = rospy.Timer(rospy.Duration(0.001), self.callback)
        rospy.Timer(rospy.Duration(3), self.octoplanner)

    def callback(self, event):
        if not self.latest_pointcloud:
            # rospy.logwarn("There is no pointcloud to navigate\n")
            return
        near_left, near_front, near_right, far_left, far_right = self.get_distances_from_sectors(self.latest_pointcloud)
        self.decide_action(near_left, near_front, near_right, far_left, far_right)
        self.latest_pointcloud = None

    def octoplanner(self, event):
        current_position = self.get_xyz_from_tf()
        if current_position is None:
            return

        if not self.is_moved(current_position):
            rospy.loginfo("not moved")
            #self.clear_octomap()
            if len(self.traveled_path) != 0 and len(self.traveled_path) != 1:
                self.traveled_path.pop()
                latest_position = self.traveled_path.pop()
                self.send_goto(latest_position[0], latest_position[1], latest_position[2], self.yaw_octogoal)
            else:
                self.send_velocity_command(vx=-0.4, vy=0.0, vz=0.0, yaw_rate=0.0)

            return

        self.traveled_path.append(current_position)

        rospy.loginfo(self.x_octogoal)
        self.send_goto(current_position[0] + self.x_octogoal,
                       current_position[1] + self.y_octogoal,
                       current_position[2],
                       self.yaw_octogoal)

        self.last_pos = current_position

    def clear_octomap(self):
        try:
            rospy.wait_for_service("/uav1/octomap_server/reset_map", timeout=1.0)
            clear_srv = rospy.ServiceProxy("/uav1/octomap_server/reset_map", Empty)
            clear_srv()
            rospy.logwarn("reset map")
        except Exception as e:
            rospy.logwarn("reset map failed")

    def send_goto(self, x, y, z, yaw):
        req = Vec4Request()
        req.goal[0] = x
        req.goal[1] = y
        req.goal[2] = z
        req.goal[3] = yaw
        try:
            resp = self.goto_srv(req)
            rospy.loginfo(f"Service response: {resp}, req: {req}")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def is_moved(self, current_position):
        if self.last_pos is None:
            self.last_pos = current_position
            return True

        dx = current_position[0] - self.last_pos[0]
        dy = current_position[1] - self.last_pos[1]
        dz = current_position[2] - self.last_pos[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        if dist < self.still_eps:
            return False
        else:
            return True

    def get_xyz_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.body_frame, rospy.Time(0), rospy.Duration(0.2))
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF lookup failed: %s" % e)
        return None

    def pointcloud_callback(self, cloud_msg):
        raw_pointcloud = self.pointcloud.read_pointcloud(cloud_msg)
        self.latest_pointcloud = self.pointcloud.change_points_frame(self.camera_frame, cloud_msg.header.frame_id,
                                                                     self.camera_frame, raw_pointcloud,
                                                                     cloud_msg.header.stamp)

    def decide_action(self, near_left, near_front, near_right, far_left, far_right):
        rospy.loginfo(
            f"\n\nDISTS: L = {near_left:.2f}   F = {near_front:.2f}  R = {near_right:.2f}  WL = {far_left:.2f}  WR = {far_right:.2f} ")

        if near_front <= self.near_threshold and near_front >= self.drone_size:
            if far_left > far_right:
                rospy.loginfo("Obstacle ahead! Turning left")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.6)
            else:
                rospy.loginfo("Obstacle ahead! Turning right")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=-0.6)
        elif near_front <= self.far_threshold:
            if near_left > near_right:
                rospy.loginfo("Obstacle ahead (far), gently veering left")
                self.send_velocity_command(vx=0.1, vy=0.1, vz=0.0, yaw_rate=0.2)
            else:
                rospy.loginfo("Obstacle ahead (far), gently veering right")
                self.send_velocity_command(vx=0.1, vy=-0.1, vz=0.0, yaw_rate=-0.2)
        else:
            rospy.loginfo("Path clear, flying straight")
            self.send_velocity_command(vx=0.4, vy=0.0, vz=0.0, yaw_rate=0.0)

    def get_distances_from_sectors(self, pointcloud):
        near_left = near_front = near_right = far_left = far_right = float('inf')
        for point in pointcloud:
            x, y, z = point
            if not (-0.15 < z < 0.15):
                continue

            angle_rad = np.arctan2(y, x)
            angle_deg = np.degrees(angle_rad)

            if angle_deg < -90 or angle_deg > 90:
                continue

            distance = np.sqrt(x ** 2 + y ** 2)

            if -20 <= angle_deg < 20:
                near_front = min(near_front, distance)
            elif 20 <= angle_deg < 30:
                near_left = min(near_left, distance)
                far_left = min(far_left, distance)
            elif -30 <= angle_deg < -20:
                near_right = min(near_right, distance)
                far_right = min(far_right, distance)
            elif 20 <= angle_deg < 90:
                far_left = min(far_left, distance)
            elif -90 <= angle_deg < -20:
                far_right = min(far_right, distance)

        return near_left, near_front, near_right, far_left, far_right

    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        msg = VelocityReferenceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.camera_frame

        msg.reference.velocity.x = vx
        msg.reference.velocity.y = vy
        msg.reference.velocity.z = vz

        msg.reference.use_heading_rate = True
        msg.reference.heading_rate = yaw_rate

        self.pub_velocity.publish(msg)


if __name__ == '__main__':
    node_instance = NavigationControllerNode()
    rospy.spin()

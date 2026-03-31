#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import VelocityReferenceStamped
from mrs_msgs.srv import Vec4
from std_srvs.srv import Empty
import tf2_ros
import numpy as np
from collections import deque

from monodepth.pointcloud_processor import PointCloudProcessor


class NavigationControllerNode(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.init_params()
        self.init_tf()
        self.init_state()
        self.init_publishers()
        self.init_subscribers()
        self.init_services()
        self.init_timers()
        self.get_logger().info("Navigation Node Started")

    def init_params(self):

        self.declare_parameter("target_frame", "fcu_untilted")
        self.declare_parameter("input_pointcloud_topic", "/midas/pointcloud_by_map")
        self.declare_parameter("output_velocity_topic", "control_manager/velocity_reference")

        self.declare_parameter("world_frame", "local_origin")
        self.declare_parameter("body_frame", "fcu_untilted")

        self.declare_parameter("x_octogoal", 5.0)
        self.declare_parameter("y_octogoal", 0.0)
        self.declare_parameter("z_octogoal", 2.0)
        self.declare_parameter("yaw_octogoal", 0.0)

        self.declare_parameter("is_reactive", False)


        self.is_reactive = self.get_parameter("is_reactive").value  # if True - navigation mode set to reactive navigation else navigation mode set to octomap planer


        self.camera_frame = self.get_parameter("target_frame").value
        self.input_pointcloud_topic = self.get_parameter("input_pointcloud_topic").value
        self.output_velocity_topic = self.get_parameter("output_velocity_topic").value

        self.world_frame = self.get_parameter("world_frame").value
        self.body_frame = self.get_parameter("body_frame").value

        self.x_octogoal = self.get_parameter("x_octogoal").value
        self.y_octogoal = self.get_parameter("y_octogoal").value
        self.z_octogoal = self.get_parameter("z_octogoal").value
        self.yaw_octogoal = self.get_parameter("yaw_octogoal").value

    def init_tf(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def init_state(self):
        self.drone_size = 0.02
        self.near_threshold = 3.0
        self.far_threshold = 12.0


        self.yaw_gain = 1.5
        self.target_reached_dist = 2.0
        self.current_position = None

        self.still_eps = 0.05
        self.last_pos = None
        self.traveled_path = deque(maxlen=10)
        self.pointcloud = PointCloudProcessor(self.tf_buffer)
        self.latest_pointcloud = None

    def init_publishers(self):
        self.pub_velocity = self.create_publisher(VelocityReferenceStamped, self.output_velocity_topic, 10)

    def init_subscribers(self):
        self.create_subscription(PointCloud2, self.input_pointcloud_topic, self.pointcloud_callback, 1)

    def init_services(self):
        ns = self.get_namespace()
        if ns == '/':
            ns = ''

        goto_topic = f"{ns}/octomap_planner/goto"
        clear_topic = f"{ns}/octomap_server/reset_map"

        self.goto_srv = self.create_client(Vec4, goto_topic)
        self.clear_srv = self.create_client(Empty, clear_topic)

        if not self.goto_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Octomap goto service not available yet!")

    def init_timers(self):
        if self.is_reactive:
            self.reactive_timer = self.create_timer(0.001, self.reactive_navigation)
        else:
            self.create_timer(3.0, self.octoplanner)

    def reactive_navigation(self):
        if not self.latest_pointcloud:
            return
        near_left, near_front, near_right, far_left, far_right = self.get_distances_from_sectors(self.latest_pointcloud)
        self.decide_action(near_left, near_front, near_right, far_left, far_right)
        self.latest_pointcloud = None

    def octoplanner(self):
        current_position = self.get_xyz_from_tf()
        if current_position is None:
            return

        if not self.is_moved(current_position):
            self.get_logger().info("Not moved")
            if len(self.traveled_path) > 1:
                self.traveled_path.pop()
                latest_position = self.traveled_path.pop()
                self.send_goto(latest_position[0], latest_position[1], latest_position[2], self.yaw_octogoal)
            else:
                self.send_velocity_command(vx=-0.4, vy=0.0, vz=0.0, yaw_rate=0.0)
            return

        self.traveled_path.append(current_position)


        target_x = self.x_octogoal
        target_y = self.y_octogoal
        target_z = self.z_octogoal

        self.get_logger().info(f"Target X={target_x}, Y={target_y}, Z={target_z}")
        self.send_goto(target_x, target_y, target_z, self.yaw_octogoal)

        # self.send_goto(current_position[0] + self.x_octogoal,
        #                current_position[1] + self.y_octogoal,
        #                current_position[2],
        #                self.yaw_octogoal)
        self.last_pos = current_position

    def clear_octomap(self):
        if self.clear_srv.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.clear_srv.call_async(req)
            self.get_logger().warn("Reset map called")
        else:
            self.get_logger().warn("Reset map service not available")

    def send_goto(self, x, y, z, yaw):
        if not self.goto_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Goto service not available")
            return

        req = Vec4.Request()
        req.goal[0] = float(x)
        req.goal[1] = float(y)
        req.goal[2] = float(z)
        req.goal[3] = float(yaw)

        # Асинхронный вызов, чтобы не заблокировать таймер
        future = self.goto_srv.call_async(req)
        future.add_done_callback(self.goto_response_callback)

    def goto_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Goto service response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def is_moved(self, current_position):
        if self.last_pos is None:
            self.last_pos = current_position
            return True

        dx = current_position[0] - self.last_pos[0]
        dy = current_position[1] - self.last_pos[1]
        dz = current_position[2] - self.last_pos[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        return dist >= self.still_eps

    def get_xyz_from_tf(self):
        try:
            now = rclpy.time.Time()
            tf_msg = self.tf_buffer.lookup_transform(self.world_frame, self.body_frame, now,
                                                     rclpy.duration.Duration(seconds=0.2))
            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation

            siny_cosp = 2 * (r.w * r.z + r.x * r.y)
            cosy_cosp = 1 - 2 * (r.y * r.y + r.z * r.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return (t.x, t.y, t.z, yaw)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
        return None

    def pointcloud_callback(self, cloud_msg):
        raw_pointcloud = self.pointcloud.read_pointcloud(cloud_msg)
        self.latest_pointcloud = self.pointcloud.change_points_frame(
            self.camera_frame, cloud_msg.header.frame_id, self.camera_frame, raw_pointcloud, cloud_msg.header.stamp)

    def decide_action(self, near_left, near_front, near_right, far_left, far_right):
        self.get_logger().info(
            f"DISTS: L={near_left:.2f} F={near_front:.2f} R={near_right:.2f} WL={far_left:.2f} WR={far_right:.2f}")

        current_position = self.get_xyz_from_tf()
        if current_position is None:
            current_position = self.current_position

        if current_position is None:
            return

        curr_x, curr_y, curr_z, curr_yaw = current_position

        target_x = self.x_octogoal
        target_y = self.y_octogoal

        dx = target_x - curr_x
        dy = target_y - curr_y
        dist_to_goal = np.sqrt(dx ** 2 + dy ** 2)

        target_yaw_global = np.arctan2(dy, dx)

        yaw_error = target_yaw_global - curr_yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        if self.near_threshold >= near_front >= self.drone_size:
            if far_left > far_right:
                self.get_logger().info("Obstacle ahead! Turning left")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.6)
            else:
                self.get_logger().info("Obstacle ahead! Turning right")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=-0.6)
        elif near_front <= self.far_threshold:
            if near_left > near_right:
                self.get_logger().info("Obstacle ahead (far), gently veering left")
                self.send_velocity_command(vx=0.1, vy=0.1, vz=0.0, yaw_rate=0.2)
            else:
                self.get_logger().info("Obstacle ahead (far), gently veering right")
                self.send_velocity_command(vx=0.1, vy=-0.1, vz=0.0, yaw_rate=-0.2)
        else:
            if dist_to_goal < 2.0:
                self.get_logger().info("Goal achieved, staying still")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
                return

            if abs(yaw_error) < 0.05:
                cmd_yaw_rate = 0.0
            else:
                cmd_yaw_rate = np.clip(self.yaw_gain * yaw_error, -0.8, 0.8)

            self.send_velocity_command(vx=0.4, vy=0.0, vz=0.0, yaw_rate=cmd_yaw_rate)
            self.get_logger().info(f"Path clear, flying straight with yaw: {cmd_yaw_rate}")

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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame

        msg.reference.velocity.x = float(vx)
        msg.reference.velocity.y = float(vy)
        msg.reference.velocity.z = float(vz)

        msg.reference.use_heading_rate = True
        msg.reference.heading_rate = float(yaw_rate)

        self.pub_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
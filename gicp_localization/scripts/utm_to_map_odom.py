#!/usr/bin/env python3
"""Republish UTM-frame GNSS odometry in the GLIM map frame.

The prepped bags (scripts/prep_bag.py) carry /gps_p1/filtered_odom with
header.frame_id="utm" (absolute UTM coordinates). The localization node
consumes its gt_odom topic directly in the *map* frame (the frame of the
PCD map built by GLIM), so during replay this node bridges the two using
the T_world_utm.txt that GLIM wrote next to the map dump:

    p_map = T_world_utm * p_utm

Usage (alongside `ros2 bag play` and the localization launch):

    python3 utm_to_map_odom.py --ros-args \
        -p utm_transform_path:=/path/to/dump/T_world_utm.txt \
        -p input_topic:=/gps_p1/filtered_odom \
        -p output_topic:=/gps_p1/filtered_odom_map

Then pass gt_odom_topic:=/gps_p1/filtered_odom_map to the localization launch.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def load_t_world_utm(path):
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "T_world_utm" in line:
                continue
            vals = line.split()
            if len(vals) == 4:
                rows.append([float(v) for v in vals])
    if len(rows) != 4:
        raise RuntimeError(f"malformed T_world_utm file ({len(rows)} matrix rows): {path}")
    return np.array(rows)


def quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def mat_to_quat(m):
    # Shepperd's method: pick the largest diagonal term for numerical stability
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] >= m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


class UtmToMapOdom(Node):
    def __init__(self):
        super().__init__("utm_to_map_odom")
        self.declare_parameter("utm_transform_path", "")
        self.declare_parameter("input_topic", "/gps_p1/filtered_odom")
        self.declare_parameter("output_topic", "/gps_p1/filtered_odom_map")
        path = self.get_parameter("utm_transform_path").value
        if not path:
            raise RuntimeError("utm_transform_path parameter is required")
        T = load_t_world_utm(path)
        self.R = T[:3, :3]
        self.t = T[:3, 3]
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(Odometry, out_topic, 50)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 50)
        self.get_logger().info(
            f"republishing {in_topic} (utm) -> {out_topic} (map); "
            f"T_world_utm t=[{self.t[0]:.2f} {self.t[1]:.2f} {self.t[2]:.2f}]")

    def cb(self, msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = msg.child_frame_id
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        pm = self.R @ p + self.t
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pm
        q = msg.pose.pose.orientation
        Rm = self.R @ quat_to_mat(q.x, q.y, q.z, q.w)
        qx, qy, qz, qw = mat_to_quat(Rm)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        # rotate position and orientation covariance blocks
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        cov[:3, :3] = self.R @ cov[:3, :3] @ self.R.T
        cov[3:, 3:] = self.R @ cov[3:, 3:] @ self.R.T
        out.pose.covariance = cov.flatten().tolist()
        out.twist = msg.twist  # body-frame twist is frame-invariant here
        self.pub.publish(out)


def main():
    rclpy.init()
    node = UtmToMapOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""RTK-FIXED odometry pre-filter for GLIM mapping.

Subscribes to an Atlas FusionEngine INS odometry topic (default
/gps_p1/filtered_odom) and republishes ONLY samples whose pose covariance
indicates RTK-FIXED quality.  The downstream consumer (GLIM's INS odometry
estimator) then gates on the presence of messages on the filtered topic --
no message means GLIM skips the LiDAR scan and the map does not grow.

Why a separate node rather than a flag inside GLIM:
* GLIM's libodometry_estimation_ins.so has no awareness of RTK status; it
  only checks temporal coverage of its INS buffer.  Filtering upstream is
  the smallest, most surgical change.

DEPRECATION / SCOPE NOTE (2026-07-14):
* This node is a LEGACY fallback. The adapter now publishes
  /gps_p1/filtered_odom_rtk_fixed with its OWN native gate
  (adapter::posePassesRtkGate), which additionally requires
  solution_type == RTK_FIXED. Prefer subscribing GLIM to the adapter's
  topic directly rather than running this node.
* This node can only gate on pose COVARIANCE: the FusionEngine
  solution_type is stripped by the adapter (nav_msgs/Odometry carries no
  such field), so this gate is strictly weaker than the adapter's and the
  two are NOT identical policies. Do not describe them as one policy.
* Parity fix: the covariance test below now matches the C++ rtk_gate
  (finite AND non-negative AND <= threshold). The former bare `<=` admitted
  a negative "covariance not populated" sentinel (-1 <= 0.001) as FIXED.

Threshold rationale (default values):
* Atlas RTK-FIXED on the AV-24 publishes cov_xx in the ~1e-6 -- 1e-4 m^2
  band (median 2.8e-5 over a 17-minute bag).  cov_zz median 1.0e-4.
* RTK-FLOAT typically lives in 1e-2 -- 1e-1 m^2 (one to two orders of
  magnitude looser).
* GPS-only is 1+ m^2.
* Setting the cutoff at 1e-3 m^2 (~3 cm horizontal std) admits FIXED
  comfortably and rejects FLOAT or worse.  For Z we allow 5e-3 m^2
  (~7 cm) because the antenna-line geometry makes Z covariance
  naturally looser than X/Y.

Run:
    python3 gicp_localization/scripts/rtk_fixed_odom_filter.py \
        --ros-args \
        -p input_topic:=/gps_p1/filtered_odom \
        -p output_topic:=/gps_p1/filtered_odom_rtk_fixed \
        -p max_pose_var_xy:=0.001 \
        -p max_pose_var_z:=0.005

Behavior at startup:
    The node logs every fix-state transition (NOT_FIXED -> FIXED and the
    reverse) so the operator can see RTK acquisition and any drop-outs
    during the mapping session.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class RtkFixedOdomFilter(Node):
    def __init__(self):
        super().__init__("rtk_fixed_odom_filter")

        self.declare_parameter("input_topic", "/gps_p1/filtered_odom")
        self.declare_parameter("output_topic", "/gps_p1/filtered_odom_rtk_fixed")
        # Pose-covariance gates (in m^2).  Tighter than gicp_localization's
        # gt_odom gate by design: GLIM mapping needs RTK-FIXED, GICP
        # localization tolerates RTK-FLOAT.
        self.declare_parameter("max_pose_var_xy", 0.001)   # ~3 cm horizontal std
        self.declare_parameter("max_pose_var_z", 0.005)    # ~7 cm vertical std
        # Log every transition by default; flip to false for noisy logs.
        self.declare_parameter("log_transitions", True)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.max_var_xy = self.get_parameter("max_pose_var_xy").get_parameter_value().double_value
        self.max_var_z = self.get_parameter("max_pose_var_z").get_parameter_value().double_value
        self.log_transitions = self.get_parameter("log_transitions").get_parameter_value().bool_value

        # Match Atlas's QoS so we don't drop messages on profile mismatch.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Odometry, output_topic, qos)
        self.sub = self.create_subscription(
            Odometry, input_topic, self.on_odom, qos)

        self.was_fixed = None
        self.received_total = 0
        self.passed_total = 0
        self.rejected_total = 0

        self.get_logger().info(
            f"RTK-FIXED odometry pre-filter ready: "
            f"{input_topic!r} -> {output_topic!r}; "
            f"max_pose_var_xy={self.max_var_xy:.4g} m^2  "
            f"max_pose_var_z={self.max_var_z:.4g} m^2")

    def on_odom(self, msg: Odometry):
        self.received_total += 1
        cov_xx = msg.pose.covariance[0]
        cov_yy = msg.pose.covariance[7]
        cov_zz = msg.pose.covariance[14]

        # Parity with the C++ rtk_gate: finite AND non-negative AND <= threshold.
        # A bare `<=` used to admit a negative "covariance not populated"
        # sentinel (-1) as FIXED.
        def _ok(v, mx):
            return math.isfinite(v) and 0.0 <= v <= mx

        is_fixed = (
            _ok(cov_xx, self.max_var_xy) and
            _ok(cov_yy, self.max_var_xy) and
            _ok(cov_zz, self.max_var_z)
        )

        if is_fixed:
            self.pub.publish(msg)
            self.passed_total += 1
        else:
            self.rejected_total += 1

        # Transition logging: print once when state flips.
        if self.log_transitions and is_fixed != self.was_fixed:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.was_fixed is None:
                # First message ever
                self.get_logger().info(
                    f"First INS sample received at stamp={stamp:.3f}: "
                    f"cov=[{cov_xx:.4g}, {cov_yy:.4g}, {cov_zz:.4g}] "
                    f"-> {'FIXED' if is_fixed else 'NOT FIXED'}")
            else:
                self.get_logger().info(
                    f"RTK transition: {'NOT_FIXED -> FIXED' if is_fixed else 'FIXED -> NOT_FIXED'} "
                    f"at stamp={stamp:.3f}  "
                    f"cov=[{cov_xx:.4g}, {cov_yy:.4g}, {cov_zz:.4g}]  "
                    f"(received={self.received_total}, passed={self.passed_total}, "
                    f"rejected={self.rejected_total})")
            self.was_fixed = is_fixed


def main():
    rclpy.init()
    node = RtkFixedOdomFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

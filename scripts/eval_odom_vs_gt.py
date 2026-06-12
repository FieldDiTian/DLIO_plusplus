#!/usr/bin/env python3
"""Compare two recorded odometry topics (e.g. localization output vs GT).

Typical use: during a localization replay, record

    ros2 bag record -o /path/loc_eval /gicp/localization/odom /gps_p1/filtered_odom_map

then run

    python3 scripts/eval_odom_vs_gt.py --bag /path/loc_eval \
        --est-topic /gicp/localization/odom --gt-topic /gps_p1/filtered_odom_map

Both topics must be in the same frame (map). Reports position error stats and
the estimated-topic publish rate (the localizer should publish at IMU rate,
~99 Hz).
"""

import argparse
import sys

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def read_odom(bag_path, topic, max_var_xy=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    dropped = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, Odometry)
        if max_var_xy is not None and max(m.pose.covariance[0], m.pose.covariance[7]) > max_var_xy:
            dropped += 1
            continue
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    if dropped:
        print(f"{topic}: dropped {dropped} samples above covariance gate")
    if not rows:
        sys.exit(f"no usable messages on {topic} in {bag_path}")
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--est-topic", default="/gicp/localization/odom")
    ap.add_argument("--gt-topic", default="/gps_p1/filtered_odom_map")
    ap.add_argument("--gt-max-var-xy", type=float, default=1e-3,
                    help="m^2; GT samples above this horizontal pose variance are excluded "
                         "(matches the localizer's own RTK quality gating). Set <=0 to disable.")
    ap.add_argument("--max-gap", type=float, default=0.5,
                    help="drop comparisons where bracketing GT samples are further apart (s)")
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    est = read_odom(args.bag, args.est_topic)
    gt = read_odom(args.bag, args.gt_topic,
                   max_var_xy=args.gt_max_var_xy if args.gt_max_var_xy > 0 else None)

    dt = np.diff(est[:, 0])
    dt = dt[dt > 0]
    if len(dt) < 2:
        sys.exit(f"{args.est_topic}: not enough distinct-stamp messages ({len(est)})")
    print(f"{args.est_topic}: n={len(est)} median publish dt={np.median(dt) * 1e3:.1f} ms "
          f"(~{1.0 / np.median(dt):.0f} Hz)")

    t0, t1 = gt[0, 0], gt[-1, 0]
    sel = (est[:, 0] >= t0) & (est[:, 0] <= t1)
    ts = est[sel, 0]
    if len(ts) < 10:
        sys.exit("est/gt stamps barely overlap")
    p_est = est[sel, 1:4]
    p_gt = np.column_stack([np.interp(ts, gt[:, 0], gt[:, i + 1]) for i in range(3)])

    # drop samples inside GT gaps (RTK dropouts / gated-out stretches)
    idx = np.searchsorted(gt[:, 0], ts).clip(1, len(gt) - 1)
    gap = gt[idx, 0] - gt[idx - 1, 0]
    ok = gap <= args.max_gap
    if (~ok).any():
        print(f"dropping {(~ok).sum()} samples inside GT gaps > {args.max_gap}s")
    ts, p_est, p_gt = ts[ok], p_est[ok], p_gt[ok]
    if len(ts) < 10:
        sys.exit("too few samples after gap filtering")

    err = p_est - p_gt
    e2d = np.linalg.norm(err[:, :2], axis=1)
    e3d = np.linalg.norm(err, axis=1)

    def stats(e):
        return (f"rms={np.sqrt(np.mean(e ** 2)):.3f}  median={np.median(e):.3f}  "
                f"p95={np.percentile(e, 95):.3f}  max={np.max(e):.3f}")

    print(f"samples compared: {len(ts)}  (window {ts[-1] - ts[0]:.1f}s)")
    print(f"horizontal error [m]: {stats(e2d)}")
    print(f"3D error         [m]: {stats(e3d)}")

    if args.csv:
        np.savetxt(args.csv, np.column_stack([ts, err, e2d, e3d]),
                   header="stamp ex ey ez e2d e3d", comments="")
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()

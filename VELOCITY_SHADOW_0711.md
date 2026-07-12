# Velocity Shadow Experiment — 2026-07-11

## Status

This branch is an archived experiment. It must not be used as the original
`ucb-roar` GICP baseline and should not be merged as a complete change set.

The velocity-shadow implementation was added by these two commits:

- `eb237a5` — small_gicp (`GICP_plusplus`) fail-closed velocity-shadow gate.
- `54638c2` — NanoGICP (`gicp_localization`) velocity-shadow gate.

Earlier commits on the branch contain separate replay, deskew, LiDAR timing,
map-contract, and parameter work. Those changes must be reviewed or ported
independently instead of treating this experiment branch as one merge unit.

## Intended behavior

The gate anchors a position track at an explicit known-pose event, propagates
that track using the body-frame velocity and orientation carried by the Atlas
odometry stream, and rejects GICP candidates whose horizontal position differs
from the propagated track by more than the configured threshold.

## Independence problem

The implementation consumes attitude and `velflu` from
`/gps_p1/filtered_odom`. Both fields originate from the Atlas FusionEngine pose
solution: attitude is the fused navigation attitude and `velflu` is the fused
body-frame navigation velocity. They are not raw wheel-speed measurements.

The Run 3 and Run 5 Point One PCAP audits contained no FusionEngine wheel-speed,
vehicle-speed, or wheel-tick input/output messages. The raw ROS bags do contain
vehicle wheel-speed topics, but no software path was found that feeds those
topics into Atlas. Therefore the tested velocity-shadow signal is correlated
with the GNSS/INS solution and cannot be treated as an independent observation
when evaluating GICP behavior during GNSS degradation.

## Disposition

- Keep this branch only as the record of the 2026-07-11 experiment.
- Do not use results 24–27 as the original `ucb-roar` baseline.
- Do not enable this gate in a GNSS-dropout evaluation unless its velocity
  source is replaced by, or conclusively verified as, an independent source
  such as vehicle CAN or hardware wheel ticks.
- Use the separate original-`ucb-roar` baseline branch and its four full-run
  reports for baseline publication.

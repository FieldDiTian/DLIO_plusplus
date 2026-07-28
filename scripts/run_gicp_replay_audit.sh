#!/usr/bin/env bash
# Run a lossless, auditable offline GICP replay.
#
# The runner is dataset-independent. It derives DATASET_ROOT from --map-dir or
# --map when possible and writes to DATASET_ROOT/gicp_result unless --out-root
# is supplied. Unaudited runs default to DATASET_ROOT/gicp_result/intermediate.
# Multiple --bag arguments are passed to ros2 bag play as inputs.
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  printf '%s\n' \
    'Usage: run_gicp_replay_audit.sh [options]' \
    '' \
    'Required:' \
    '  --map FILE | --map-dir DIR   ENU PCD or directory containing map.pcd' \
    '  --bag PATH                   rosbag2 input; repeat for multiple inputs' \
    '  --run-name NAME              new result directory name' \
    '  --overlay SETUP.BASH         built GICP++ overlay' \
    '  --duration SECONDS           playback duration' \
    '' \
    'Common options:' \
    '  --out-root DIR               defaults to DATASET_ROOT/gicp_result/intermediate' \
    '  --start-offset SECONDS       default 0' \
    '  --rate RATE                  default 1.0' \
    '  --domain-id ID               default 177' \
    '  --pointcloud-topic TOPIC     default /luminar_front/points' \
    '  --imu-topic TOPIC            default /gps_p1/imu' \
    '  --gt-topic TOPIC             default /gps_p1/filtered_odom' \
    '  --reference-topic TOPIC      defaults to --gt-topic' \
    '  --primary-queue-size N       default 8' \
    '  --read-ahead-queue-size N    rosbag playback prefetch; default 50000' \
    '  --config-path YAML           run-local overrides loaded after package defaults' \
    '  --qos-overrides YAML         optional publisher QoS override' \
    '  --play-topic TOPIC           repeat to replace the default topic set' \
    '  --bridge-script FILE         optional preprocessing/offset ROS node' \
    '  --bridge-arg VALUE           repeat; passed literally to the bridge'
}

MAP=
MAP_DIR=
OUT_ROOT=
RUN_NAME=
OVERLAY=
START_OFFSET=0
DURATION=
RATE=1.0
DOMAIN_ID=177
STORAGE_ID=mcap
POINTCLOUD_TOPIC=/luminar_front/points
IMU_TOPIC=/gps_p1/imu
GT_TOPIC=/gps_p1/filtered_odom
REFERENCE_TOPIC=
PRIMARY_QUEUE_SIZE=8
READ_AHEAD_QUEUE_SIZE=50000
CONFIG_PATH=
FUTURE_AUX_WAIT_TIMEOUT_S=0.150
LIDAR_CONCAT_ENABLED=false
REQUIRE_ALL_AUX=false
LIDAR_RELIABLE_QOS=true
QOS_OVERRIDES=
BRIDGE_SCRIPT=
declare -a BAGS=()
declare -a BRIDGE_ARGS=()
declare -a PLAY_TOPICS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:?missing value for --map}"; shift 2 ;;
    --map-dir) MAP_DIR="${2:?missing value for --map-dir}"; shift 2 ;;
    --bag) BAGS+=("${2:?missing value for --bag}"); shift 2 ;;
    --out-root) OUT_ROOT="${2:?missing value for --out-root}"; shift 2 ;;
    --run-name) RUN_NAME="${2:?missing value for --run-name}"; shift 2 ;;
    --overlay) OVERLAY="${2:?missing value for --overlay}"; shift 2 ;;
    --start-offset) START_OFFSET="${2:?missing value for --start-offset}"; shift 2 ;;
    --duration) DURATION="${2:?missing value for --duration}"; shift 2 ;;
    --rate) RATE="${2:?missing value for --rate}"; shift 2 ;;
    --domain-id) DOMAIN_ID="${2:?missing value for --domain-id}"; shift 2 ;;
    --storage-id) STORAGE_ID="${2:?missing value for --storage-id}"; shift 2 ;;
    --pointcloud-topic) POINTCLOUD_TOPIC="${2:?missing value}"; shift 2 ;;
    --imu-topic) IMU_TOPIC="${2:?missing value}"; shift 2 ;;
    --gt-topic) GT_TOPIC="${2:?missing value}"; shift 2 ;;
    --reference-topic) REFERENCE_TOPIC="${2:?missing value}"; shift 2 ;;
    --primary-queue-size) PRIMARY_QUEUE_SIZE="${2:?missing value}"; shift 2 ;;
    --read-ahead-queue-size) READ_AHEAD_QUEUE_SIZE="${2:?missing value}"; shift 2 ;;
    --config-path) CONFIG_PATH="${2:?missing value}"; shift 2 ;;
    --future-aux-wait-timeout) FUTURE_AUX_WAIT_TIMEOUT_S="${2:?missing value}"; shift 2 ;;
    --lidar-concat-enabled) LIDAR_CONCAT_ENABLED="${2:?missing value}"; shift 2 ;;
    --require-all-aux) REQUIRE_ALL_AUX="${2:?missing value}"; shift 2 ;;
    --lidar-reliable-qos) LIDAR_RELIABLE_QOS="${2:?missing value}"; shift 2 ;;
    --qos-overrides) QOS_OVERRIDES="${2:?missing value}"; shift 2 ;;
    --play-topic) PLAY_TOPICS+=("${2:?missing value}"); shift 2 ;;
    --bridge-script) BRIDGE_SCRIPT="${2:?missing value}"; shift 2 ;;
    --bridge-arg) BRIDGE_ARGS+=("${2:?missing value}"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$MAP" && -n "$MAP_DIR" ]]; then
  printf 'Pass only one of --map or --map-dir\n' >&2
  exit 2
fi
if [[ -n "$MAP_DIR" ]]; then
  MAP="${MAP_DIR%/}/map.pcd"
fi
if [[ -z "$MAP" || -z "$RUN_NAME" || -z "$OVERLAY" || -z "$DURATION" ]]; then
  printf '%s\n' '--map/--map-dir, --run-name, --overlay and --duration are required' >&2
  usage >&2
  exit 2
fi
if [[ ${#BAGS[@]} -eq 0 ]]; then
  printf 'At least one --bag is required\n' >&2
  exit 2
fi
if [[ ! "$PRIMARY_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ||
      ! "$READ_AHEAD_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Queue sizes must be positive integers\n' >&2
  exit 2
fi

MAP="$(realpath -e "$MAP")"
OVERLAY="$(realpath -e "$OVERLAY")"
for index in "${!BAGS[@]}"; do
  BAGS[$index]="$(realpath -e "${BAGS[$index]}")"
done
if [[ -n "$QOS_OVERRIDES" ]]; then
  QOS_OVERRIDES="$(realpath -e "$QOS_OVERRIDES")"
fi
if [[ -n "$BRIDGE_SCRIPT" ]]; then
  BRIDGE_SCRIPT="$(realpath -e "$BRIDGE_SCRIPT")"
fi
if [[ -n "$CONFIG_PATH" ]]; then
  CONFIG_PATH="$(realpath -e "$CONFIG_PATH")"
fi

if [[ "$MAP" == */maps/* ]]; then
  DATASET_ROOT="${MAP%%/maps/*}"
elif [[ "$(basename "$(dirname "$MAP")")" == "maps" ]]; then
  DATASET_ROOT="$(dirname "$(dirname "$MAP")")"
else
  DATASET_ROOT=
fi
if [[ -z "$OUT_ROOT" ]]; then
  if [[ -z "$DATASET_ROOT" ]]; then
    printf 'Could not derive DATASET_ROOT from map path; pass --out-root explicitly\n' >&2
    exit 2
  fi
  OUT_ROOT="$DATASET_ROOT/gicp_result/intermediate"
fi
OUT_ROOT="$(realpath -m "$OUT_ROOT")"
RUN_DIR="$OUT_ROOT/$RUN_NAME"

if [[ -e "$RUN_DIR" ]]; then
  printf 'Refusing to overwrite run directory: %s\n' "$RUN_DIR" >&2
  exit 3
fi
if [[ ! -s "$MAP" ]]; then
  printf 'Map is missing or empty: %s\n' "$MAP" >&2
  exit 3
fi
if [[ "$LIDAR_RELIABLE_QOS" == "true" && -z "$QOS_OVERRIDES" ]]; then
  QOS_OVERRIDES="$SCRIPT_DIR/../GICP_plusplus/cfg/lidar_reliable_replay.yaml"
fi
if [[ "$LIDAR_RELIABLE_QOS" == "true" && ! -s "$QOS_OVERRIDES" ]]; then
  printf 'Reliable replay QoS file is missing or empty: %s\n' "$QOS_OVERRIDES" >&2
  exit 3
fi
if [[ -z "$REFERENCE_TOPIC" ]]; then
  REFERENCE_TOPIC="$GT_TOPIC"
fi
if [[ ${#PLAY_TOPICS[@]} -eq 0 ]]; then
  PLAY_TOPICS=(
    "$POINTCLOUD_TOPIC"
    /luminar_left/points
    /luminar_right/points
    "$IMU_TOPIC"
    "$GT_TOPIC"
  )
  if [[ "$REFERENCE_TOPIC" != "$GT_TOPIC" ]]; then
    PLAY_TOPICS+=("$REFERENCE_TOPIC")
  fi
fi

source /opt/ros/jazzy/setup.bash
source "$OVERLAY"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
mkdir -p "$RUN_DIR" "$ROS_LOG_DIR"

bridge_pid=
launch_pid=
record_pid=
reference_record_pid=
resource_pid=

stop_pid() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.25
    done
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

stop_launch() {
  if [[ -z "$launch_pid" ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
    return 0
  fi
  local child
  while read -r child; do
    [[ -n "$child" ]] && kill -INT "$child" 2>/dev/null || true
  done < <(pgrep -P "$launch_pid" || true)
  for _ in {1..80}; do
    pgrep -P "$launch_pid" >/dev/null 2>&1 || break
    sleep 0.25
  done
  stop_pid "$launch_pid"
}

cleanup() {
  stop_pid "$record_pid"
  stop_pid "$reference_record_pid"
  stop_launch
  stop_pid "$bridge_pid"
  stop_pid "$resource_pid"
}
trap cleanup EXIT INT TERM

if [[ -n "$BRIDGE_SCRIPT" ]]; then
  python3 "$BRIDGE_SCRIPT" "${BRIDGE_ARGS[@]}" >"$RUN_DIR/bridge.log" 2>&1 &
  bridge_pid=$!
fi

ros2 launch gicp_plusplus localization_with_tf.launch.py \
  rviz:=false \
  map_path:="$MAP" \
  pointcloud_topic:="$POINTCLOUD_TOPIC" \
  imu_topic:="$IMU_TOPIC" \
  gt_odom_topic:="$GT_TOPIC" \
  lidar_concat_enabled:="$LIDAR_CONCAT_ENABLED" \
  require_all_aux:="$REQUIRE_ALL_AUX" \
  lidar_reliable_qos:="$LIDAR_RELIABLE_QOS" \
  future_aux_wait_timeout_s:="$FUTURE_AUX_WAIT_TIMEOUT_S" \
  primary_queue_size:="$PRIMARY_QUEUE_SIZE" \
  config_path:="$CONFIG_PATH" \
  >"$RUN_DIR/localization.log" 2>&1 &
launch_pid=$!

initialized=0
for _ in {1..180}; do
  if grep -q "DLIO Localization Node Initialized" "$RUN_DIR/localization.log"; then
    initialized=1
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    printf 'Localization launch exited before initialization\n' >&2
    exit 4
  fi
  sleep 1
done
if [[ "$initialized" -ne 1 ]]; then
  printf 'Timed out waiting for localization initialization\n' >&2
  exit 5
fi

(
  while kill -0 "$launch_pid" 2>/dev/null; do
    date --iso-8601=seconds
    ps -o pid,etime,%cpu,%mem,rss,stat,cmd -C gicp_plusplus_node || true
    sleep 5
  done
) >"$RUN_DIR/resource.log" 2>&1 &
resource_pid=$!

ros2 bag record --storage mcap --output "$RUN_DIR/debug_topics_bag" \
  --regex '(^/gicp/localization/debug(/.*)?$)' \
  >"$RUN_DIR/record.log" 2>&1 &
record_pid=$!

ros2 bag record --storage mcap --output "$RUN_DIR/reference_topics_bag" \
  "$REFERENCE_TOPIC" >"$RUN_DIR/reference_record.log" 2>&1 &
reference_record_pid=$!
sleep 2

declare -a play_args=()
for bag in "${BAGS[@]}"; do
  play_args+=(-i "$bag" "$STORAGE_ID")
done
play_args+=(
  --read-ahead-queue-size "$READ_AHEAD_QUEUE_SIZE"
  --rate "$RATE"
  --start-offset "$START_OFFSET"
  --playback-duration "$DURATION"
  --clock-topics "$POINTCLOUD_TOPIC"
  --disable-keyboard-controls
  --topics
)
play_args+=("${PLAY_TOPICS[@]}")
if [[ "$LIDAR_RELIABLE_QOS" == "true" ]]; then
  play_args+=(--qos-profile-overrides-path "$QOS_OVERRIDES")
fi

play_start_ns="$(date +%s%N)"
ros2 bag play "${play_args[@]}" >"$RUN_DIR/playback.log" 2>&1
playback_exit=$?
play_end_ns="$(date +%s%N)"

sleep 3
launch_alive=0
if kill -0 "$launch_pid" 2>/dev/null; then
  launch_alive=1
fi

stop_launch
launch_pid=
sleep 2
stop_pid "$record_pid"
record_pid=
stop_pid "$reference_record_pid"
reference_record_pid=
stop_pid "$bridge_pid"
bridge_pid=
stop_pid "$resource_pid"
resource_pid=

play_wall_s="$(awk -v start="$play_start_ns" -v end="$play_end_ns" \
  'BEGIN { printf "%.6f", (end-start)/1000000000.0 }')"
{
  printf 'playback_exit=%s\n' "$playback_exit"
  printf 'localization_alive_after_playback=%s\n' "$launch_alive"
  printf 'completed_utc=%s\n' "$(date --utc --iso-8601=seconds)"
  printf 'dataset_root=%s\n' "$DATASET_ROOT"
  printf 'ros_domain_id=%s\n' "$ROS_DOMAIN_ID"
  printf 'map=%s\n' "$MAP"
  printf 'map_bytes=%s\n' "$(stat -c %s "$MAP")"
  printf 'map_sha256=%s\n' "$(sha256sum "$MAP" | awk '{print $1}')"
  printf 'bags=%s\n' "${BAGS[*]}"
  printf 'start_offset_s=%s\n' "$START_OFFSET"
  printf 'playback_duration_s=%s\n' "$DURATION"
  printf 'playback_rate=%s\n' "$RATE"
  printf 'playback_wall_s=%s\n' "$play_wall_s"
  printf 'lidar_concat_enabled=%s\n' "$LIDAR_CONCAT_ENABLED"
  printf 'require_all_aux=%s\n' "$REQUIRE_ALL_AUX"
  printf 'lidar_reliable_qos=%s\n' "$LIDAR_RELIABLE_QOS"
  printf 'future_aux_wait_timeout_s=%s\n' "$FUTURE_AUX_WAIT_TIMEOUT_S"
  printf 'primary_queue_size=%s\n' "$PRIMARY_QUEUE_SIZE"
  printf 'read_ahead_queue_size=%s\n' "$READ_AHEAD_QUEUE_SIZE"
  printf 'config_path=%s\n' "$CONFIG_PATH"
  if [[ -n "$CONFIG_PATH" ]]; then
    printf 'config_sha256=%s\n' "$(sha256sum "$CONFIG_PATH" | awk '{print $1}')"
  fi
  printf 'pointcloud_topic=%s\n' "$POINTCLOUD_TOPIC"
  printf 'imu_topic=%s\n' "$IMU_TOPIC"
  printf 'gt_topic=%s\n' "$GT_TOPIC"
  printf 'reference_topic=%s\n' "$REFERENCE_TOPIC"
} >"$RUN_DIR/run_status.env"

python3 "$SCRIPT_DIR/../GICP_plusplus/scripts/analyze_scan_debug_log.py" \
  "$RUN_DIR/localization.log" \
  >"$RUN_DIR/scan_debug_scorecard.md" \
  2>"$RUN_DIR/scan_debug_scorecard.err" || true

if [[ "$playback_exit" -ne 0 || "$launch_alive" -ne 1 ]]; then
  exit 6
fi

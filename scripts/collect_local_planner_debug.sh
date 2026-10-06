#!/usr/bin/env bash
# Read-only ROS diagnostics. Missing commands/master/topics never abort other items.
set -u

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="$repo_root/debug_logs"
config="$repo_root/src/planning/config/local_planner.yaml"
window=8
lidar_frame=velodyne
usage() {
  echo "Usage: bash $0 [--config YAML] [--output-root DIR] [--timeout SECONDS] [--lidar-frame FRAME]"
}
while (($#)); do
  case "$1" in
    --config|--output-root|--timeout|--lidar-frame)
      if (($# < 2)); then usage; exit 2; fi
      case "$1" in
        --config) config="$2" ;;
        --output-root) output_root="$2" ;;
        --timeout) window="$2" ;;
        --lidar-frame) lidar_frame="$2" ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
if [[ ! "$window" =~ ^[1-9][0-9]*$ ]] || ((window > 60)); then
  echo "ERROR: --timeout requires an integer from 1 to 60 seconds" >&2
  exit 2
fi
mkdir -p -- "$output_root" || exit 2
out="$(mktemp -d "$output_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")" || exit 2
printf 'Collected: %s\nRepository: %s\nRequested YAML: %s\nROS_MASTER_URI: %s\nWindow: %ss\n' \
  "$(date -Iseconds)" "$repo_root" "$config" "${ROS_MASTER_URI:-unavailable (not set)}" "$window" > "$out/manifest.txt"

capture() {
  local name="$1" mode="$2" rc status file raw sample_ok=0
  shift 2
  file="$out/$name.txt"
  raw="$out/$name.output"
  printf 'Command:' > "$file"
  printf ' %q' "$@" >> "$file"
  printf '\n' >> "$file"
  if ! command -v "$1" >/dev/null 2>&1; then
    printf 'STATUS: unavailable (command missing: %s)\n' "$1" >> "$file"
    return 0
  fi
  if ! command -v timeout >/dev/null 2>&1; then
    printf 'STATUS: unavailable (GNU timeout missing; unbounded command skipped)\n' >> "$file"
    return 0
  fi
  # Python ROS tools can buffer stdout when piped. Preserve partial samples at timeout.
  PYTHONUNBUFFERED=1 timeout -k 2 "${window}s" "$@" > "$raw" 2>&1
  rc=$?
  status=PASS
  if ((rc == 124 || rc == 137)); then
    if [[ "$mode" == hz ]] && grep -q 'average rate:' "$raw"; then
      sample_ok=1
    elif [[ "$mode" == tf ]] && grep -q 'Translation:' "$raw"; then
      sample_ok=1
    fi
    if ((sample_ok)) && \
       ! grep -Eiq 'ERROR|Exception|no new messages|does not appear to be published|could not find|does not exist|lookup would require' "$raw"; then
      status="PASS (bounded sample; acquisition window ended)"
    else
      status="unavailable (timed out; see partial output)"
    fi
  elif ((rc != 0)); then
    status="unavailable (exit $rc)"
  elif [[ ! -s "$raw" && "$mode" != empty_ok ]]; then
    status="unavailable (no output)"
  fi
  printf 'STATUS: %s\n' "$status" >> "$file"
  cat -- "$raw" >> "$file"
  rm -f -- "$raw"
  return 0
}

capture git_branch quick git -C "$repo_root" branch --show-current
capture git_commit quick git -C "$repo_root" rev-parse HEAD
capture git_status empty_ok git -C "$repo_root" status --short
if [[ -f "$config" ]]; then
  cp -- "$config" "$out/local_planner.yaml" || printf 'unavailable (copy failed)\n' > "$out/config_status.txt"
  capture config_sha256 quick sha256sum "$config"
else
  printf 'unavailable: requested YAML does not exist: %s\n' "$config" > "$out/config_status.txt"
fi

# Independent probes run concurrently: unreachable ROS does not multiply wait times.
for package in planning control lidar_object_detection simul_msgs main morai_udp gps camera; do
  capture "package_$package" quick rospack find "$package" &
done
capture node_list quick rosnode list &
capture topic_list quick rostopic list &
capture params_local quick rosparam get /static_obstacle_avoidance_planner &
capture params_viz quick rosparam get /viz_planner &
capture params_control quick rosparam get /control &
capture params_detector quick rosparam get /object_detection_static &
for topic in current_pose global_path control_path obstacle_info_static velodyne_points local_plan control_cmd; do
  capture "hz_$topic" hz rostopic hz "/$topic" &
done
capture local_plan_sample quick rostopic echo -n 1 /local_plan &
capture control_cmd_sample quick rostopic echo -n 1 /control_cmd &
capture tf_map_lidar tf rosrun tf tf_echo map "$lidar_frame" &
wait

printf 'item\tstatus\n' > "$out/summary.tsv"
for item in "$out"/*.txt; do
  status="$(grep -m 1 '^STATUS:' "$item" || true)"
  if [[ -n "$status" ]]; then
    printf '%s\t%s\n' "$(basename -- "$item")" "$status" >> "$out/summary.tsv"
  fi
done
printf 'Snapshot is not atomic; streams were sampled concurrently.\nRuntime params override the copied YAML; inspect both.\n' >> "$out/manifest.txt"
echo "Diagnostic folder: $out"
echo "Send this folder with docs/LOCAL_PLANNER_TEST_RESULTS.md and any rosbag/screenshots."

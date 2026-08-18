#!/bin/bash
# NITROS zero-copy benchmark, run inside the Isaac ROS container on the Jetson.
#
# Three phases, each a fresh camera node plus one consumer:
#   zerocopy  : NITROS publishing on (pooled allocator)  + nitros_consumer  (frame stays on the GPU)
#   baseline  : NITROS publishing off                    + cpu_consumer     (frame copied H2D)
#   legacy    : NITROS publishing on (cudaMalloc/frame)  + nitros_consumer  (pre-pool publisher)
#
# Each phase reports consumer FPS / latency / CPU and the camera node's own CPU over the window.
# ROS setup scripts reference unbound vars, so no set -u here.
set -o pipefail

WS=/workspaces/isaac_ros-dev
OUT=${OUT:-$WS/bench_results}
WARMUP=${WARMUP:-5.0}
DURATION=${DURATION:-30.0}
PROFILE=${PROFILE:-1920x1080x30}
FPS=${FPS:-30.0}

mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
for d in $(find $WS/install -name 'libgxf_*.so' -exec dirname {} \; | sort -u); do
  export LD_LIBRARY_PATH="$d:$LD_LIBRARY_PATH"
done

CAM_ARGS=(--ros-args
  -p "rgb_camera.color_profile:=$PROFILE"
  -p rgb_camera.color_format:=RGB8
  -p enable_depth:=false -p enable_infra1:=false -p enable_infra2:=false
  -p enable_gyro:=false -p enable_accel:=false -p pointcloud.enable:=false
  -p align_depth.enable:=false)

# The camera node's own pid, not the "ros2 run" wrapper that spawned it.
node_pid() {
  # Match on the executable itself (field 2), which excludes the python "ros2 run" wrapper and
  # any shell whose command line merely mentions the node.
  ps -eo pid,args | awk '$2 ~ /realsense2_camera_node$/ { print $1; exit }'
}

# CPU seconds consumed by a pid so far (utime+stime from /proc/<pid>/stat).
cpu_seconds() {
  local pid=$1
  [ -r "/proc/$pid/stat" ] || { echo 0; return; }
  awk '{ n=split($0, a, ") "); split(a[n], f, " "); print (f[12]+f[13])/'"$(getconf CLK_TCK)"' }' "/proc/$pid/stat"
}

run_phase() {
  local name=$1 nitros=$2 consumer=$3 legacy=$4
  local log="$OUT/$name.log"
  echo "=== phase $name (nitros=$nitros consumer=$consumer legacy_alloc=$legacy) ==="

  pkill -f realsense2_camera_node 2>/dev/null; pkill -f _consumer 2>/dev/null; sleep 3

  if [ "$legacy" = "1" ]; then export RS_NITROS_LEGACY_ALLOC=1; else unset RS_NITROS_LEGACY_ALLOC; fi
  ros2 run realsense2_camera realsense2_camera_node "${CAM_ARGS[@]}" \
      -p "enable_color_nitros:=$nitros" > "$OUT/$name.camera.log" 2>&1 &
  local cam_wrapper=$!

  # Wait for the stream to actually start before timing anything.
  local waited=0
  while ! grep -qE "Stream started|Starting Sensor" "$OUT/$name.camera.log" 2>/dev/null; do
    sleep 1; waited=$((waited+1))
    if [ $waited -gt 60 ]; then echo "TIMEOUT waiting for camera in phase $name"; cat "$OUT/$name.camera.log"|tail -20; return 1; fi
  done
  sleep 3
  local cam_pid
  cam_pid=$(node_pid)
  echo "camera pid $cam_pid (up after ${waited}s)"

  # Consumer runs warmup+duration; sample the camera's CPU over the measured part only.
  ros2 run nitros_zc_bench "$consumer" --ros-args \
      -p "warmup_s:=$WARMUP" -p "duration_s:=$DURATION" -p "expected_fps:=$FPS" \
      -p "csv_path:=$OUT/$name.frames.csv" > "$log" 2>&1 &
  local cons_wrapper=$!
  sleep "$WARMUP"
  local c0 t0
  c0=$(cpu_seconds "$cam_pid"); t0=$(date +%s.%N)
  wait $cons_wrapper
  local c1 t1
  c1=$(cpu_seconds "$cam_pid"); t1=$(date +%s.%N)
  local cam_pct
  cam_pct=$(awk -v c0="$c0" -v c1="$c1" -v t0="$t0" -v t1="$t1" 'BEGIN{ d=t1-t0; if(d>0) printf "%.2f", 100*(c1-c0)/d; else print "0" }')
  echo "PUBLISHER_CPU {\"phase\":\"$name\",\"camera_cpu_pct\":$cam_pct}" | tee -a "$log"
  grep -E "^RESULT|^PUBLISHER_CPU" "$log" | sed "s/^/[$name] /"
  # Did the zero-copy source hold for every frame?
  grep -c "ZERO-COPY source" "$OUT/$name.camera.log" 2>/dev/null | sed "s/^/[$name] zero-copy-source log lines: /"

  kill $cam_wrapper 2>/dev/null; pkill -f realsense2_camera_node 2>/dev/null; sleep 3
  return 0
}

echo "### profile=$PROFILE warmup=${WARMUP}s duration=${DURATION}s"
run_phase zerocopy true  nitros_consumer 0
run_phase baseline false cpu_consumer    0
run_phase legacy   true  nitros_consumer 1

# Both consumers against one camera node, so they see the identical frames and the latency
# difference between the two transports is measured on the same stream rather than across runs.
# (They do contend for CPU/GPU here, so treat the FPS/CPU numbers from the single-consumer phases
# as the authoritative ones and read this phase for the latency delta.)
both_phase() {
  local name=both
  echo "=== phase $name (nitros=true, cpu_consumer + nitros_consumer together) ==="
  pkill -f realsense2_camera_node 2>/dev/null; pkill -f _consumer 2>/dev/null; sleep 3
  unset RS_NITROS_LEGACY_ALLOC
  ros2 run realsense2_camera realsense2_camera_node "${CAM_ARGS[@]}" -p enable_color_nitros:=true \
      > "$OUT/$name.camera.log" 2>&1 &
  local wrapper=$!
  local waited=0
  while ! grep -qE "Stream started|Starting Sensor" "$OUT/$name.camera.log" 2>/dev/null; do
    sleep 1; waited=$((waited+1))
    if [ $waited -gt 60 ]; then echo "TIMEOUT waiting for camera in phase $name"; return 1; fi
  done
  sleep 3
  ros2 run nitros_zc_bench nitros_consumer --ros-args -p "warmup_s:=$WARMUP" \
      -p "duration_s:=$DURATION" -p "expected_fps:=$FPS" \
      -p "csv_path:=$OUT/$name.nitros.csv" > "$OUT/$name.nitros.log" 2>&1 &
  local a=$!
  ros2 run nitros_zc_bench cpu_consumer --ros-args -p "warmup_s:=$WARMUP" \
      -p "duration_s:=$DURATION" -p "expected_fps:=$FPS" \
      -p "csv_path:=$OUT/$name.cpu.csv" > "$OUT/$name.cpu.log" 2>&1 &
  local b=$!
  wait $a; wait $b
  grep -hE "^RESULT" "$OUT/$name.nitros.log" "$OUT/$name.cpu.log" | sed "s/^/[$name] /"
  python3 "$(dirname "$0")/pair_frames.py" "$OUT/$name.nitros.csv" "$OUT/$name.cpu.csv" || true
  echo "--- first frames seen by each consumer (pair by stamp_ns) ---"
  grep -h "STAMP" "$OUT/$name.nitros.log" | sed 's/^/[both:nitros] /' | tail -5
  grep -h "STAMP" "$OUT/$name.cpu.log" | sed 's/^/[both:cpu]    /' | tail -5
  kill $wrapper 2>/dev/null; pkill -f realsense2_camera_node 2>/dev/null; sleep 3
}
both_phase

# Separate short phase with debug logging on: confirms the SDK really handed us a zero-copy
# device pointer (copied == false) for every frame, and reports the publisher's own per-frame cost.
# Kept out of the measured phases because debug logging itself costs CPU.
verify_phase() {
  local name=verify log="$OUT/$name.camera.log"
  echo "=== phase $name (debug logging, 20 s) ==="
  pkill -f realsense2_camera_node 2>/dev/null; pkill -f _consumer 2>/dev/null; sleep 3
  unset RS_NITROS_LEGACY_ALLOC
  ros2 run realsense2_camera realsense2_camera_node "${CAM_ARGS[@]}" -p enable_color_nitros:=true \
      --log-level camera.camera:=debug --log-level NitrosImagePublisher:=debug > "$log" 2>&1 &
  local wrapper=$!
  sleep 12
  ros2 run nitros_zc_bench nitros_consumer --ros-args -p warmup_s:=1.0 -p duration_s:=8.0 \
      -p "expected_fps:=$FPS" > "$OUT/$name.log" 2>&1
  kill $wrapper 2>/dev/null; pkill -f realsense2_camera_node 2>/dev/null; sleep 2
  echo "VERIFY zero-copy-source frames : $(grep -c 'ZERO-COPY source' "$log")"
  echo "VERIFY upload (h2d copy) frames: $(grep -c 'UPLOAD (host->device copy)' "$log")"
  grep -h "publish cost over" "$log" | tail -3
  grep -hE "^RESULT" "$OUT/$name.log" | sed 's/^/[verify] /'
}
verify_phase

echo; echo "########## SUMMARY ##########"
grep -hE "^RESULT|^PUBLISHER_CPU" "$OUT"/*.log

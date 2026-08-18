# nitros_zc_bench

Consumer-side benchmark for the `realsense2_camera` NITROS zero-copy color path (RSDEV-6254).

Both consumers run the **identical** CUDA kernel over every byte of every frame, so the only thing
being compared is how the frame reaches the GPU:

| Executable | Subscribes to | Getting onto the GPU |
| --- | --- | --- |
| `cpu_consumer` | `~/color/image_raw` (`sensor_msgs/Image`) | `cudaMemcpy` host→device, every frame |
| `nitros_consumer` | `~/color/nitros_image` (negotiated `nitros_image_rgb8`) | nothing — the pixels are already in GPU memory |

Each run reports delivered FPS (against the stream rate), the onto-GPU cost per frame, kernel time,
transport and end-to-end latency (frame timestamp → GPU work complete), and the consumer's own CPU
as a percentage of one core. A machine-readable `RESULT {...}` line is printed at the end of a run.

## Running it

`scripts/run_bench.sh` drives the whole comparison inside the Isaac ROS container on the Jetson. It
runs four phases, each with a fresh camera node, and samples the **camera node's** CPU over the
measured window too:

| Phase | Publisher | Consumer |
| --- | --- | --- |
| `zerocopy` | `enable_color_nitros:=true`, pooled allocator | `nitros_consumer` |
| `baseline` | `enable_color_nitros:=false` | `cpu_consumer` |
| `legacy` | `enable_color_nitros:=true`, `RS_NITROS_LEGACY_ALLOC=1` (pre-pool `cudaMalloc` per frame) | `nitros_consumer` |
| `verify` | debug logging on | short run; counts `ZERO-COPY source` vs `UPLOAD` frames |

```bash
# inside the container, with the workspace built with -DBUILD_WITH_NITROS=ON
./src/nitros_zc_bench/scripts/run_bench.sh
# knobs: OUT=<dir> WARMUP=5.0 DURATION=30.0 PROFILE=1920x1080x30 FPS=30.0
```

Results land in `$OUT` (default `<workspace>/bench_results/`): one `<phase>.log` per consumer and
`<phase>.camera.log` per camera node, plus a summary of all `RESULT` / `PUBLISHER_CPU` lines.

## Looking at the GPU frame

`nitros_zc_bench::NitrosViewer` lets you see the pixels the wrapper actually published, without
undoing the zero-copy it exists to demonstrate.

```bash
ros2 launch nitros_zc_bench view_gpu_frame.launch.py            # camera + composed viewer
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nitros_zc_bench)/share/nitros_zc_bench/rviz/gpu_view.rviz
```

It subscribes to `~/color/nitros_image` **in the camera's own process**, downscales the frame on the
GPU, and copies back only a thumbnail — 480x270 (57 KB) at 2 Hz by default, against 6.22 MB at
30 Hz. It republishes that as a plain `sensor_msgs/Image` on `/nitros_viewer/gpu_view`, which rviz
(a separate process) can subscribe to harmlessly.

| Parameter | Default | Notes |
| --- | --- | --- |
| `rate_hz` | `2.0` | `0` makes the viewer inert |
| `view_width` / `view_height` | `480` / `270` | thumbnail size |
| `full_res` | `false` | copy whole frames back: fine for eyeballing, do not benchmark with it |
| `snapshot_path` | `''` | write one PPM of the GPU frame, for checking content with no display attached |

Measured cost of running it at the defaults: publisher stays at **373-396 us** per frame, versus
**372-385 us** with no viewer at all — i.e. free.

> **Do not point rviz at `~/color/nitros_image` directly.** A subscriber in another process forces
> NITROS's type adapter to copy every frame device->host and takes the publisher from ~400 us to
> ~4400 us per frame. That is the whole reason the viewer is a composed component.

## Requirements

An Isaac ROS workspace (release-3.2 on JetPack 6.x) with `isaac_ros_managed_nitros` and
`isaac_ros_nitros_image_type` built, `ros-humble-magic-enum` and `ros-humble-negotiated` installed,
and the GXF extension directories on `LD_LIBRARY_PATH` (the script handles the last part).

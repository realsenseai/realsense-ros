"""Camera + a composed viewer, so you can actually look at the GPU frame.

    ros2 launch nitros_zc_bench view_gpu_frame.launch.py
    ros2 run rviz2 rviz2   # then add an Image display on /nitros_viewer/gpu_view

The viewer is composed into the camera's container on purpose. rviz itself stays a separate
process, but it subscribes to the small `sensor_msgs/Image` the viewer republishes - NOT to
`~/color/nitros_image`. Subscribing to the NITROS topic from another process forces the type
adapter to copy every frame device->host and takes the publisher from ~400 us to ~4400 us per
frame, which defeats the whole point.

Cost of the viewer at defaults: one GPU downscale plus a 57 KB device->host copy, twice a second.
Set rate_hz:=0 to make it inert, or full_res:=true to copy whole frames back (fine for eyeballing,
useless for benchmarking).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _container(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    rate_hz = float(LaunchConfiguration('rate_hz').perform(context))
    full_res = LaunchConfiguration('full_res').perform(context).lower() == 'true'
    snapshot_path = LaunchConfiguration('snapshot_path').perform(context)
    view_width = int(LaunchConfiguration('view_width').perform(context))
    view_height = int(LaunchConfiguration('view_height').perform(context))

    return [
        ComposableNodeContainer(
            name='view_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            # One line per 100 frames of the publisher's own cost, so you can confirm the viewer
            # is not putting the removed copy back (expect ~400 us, not ~4000 us).
            arguments=['--ros-args', '--log-level', 'NitrosImagePublisher:=debug'],
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name='camera',
                    parameters=[{
                        'rgb_camera.color_profile': profile,
                        'rgb_camera.color_format': 'RGB8',
                        'enable_color_nitros': True,
                        'enable_depth': False,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_gyro': False,
                        'enable_accel': False,
                        'pointcloud.enable': False,
                        'align_depth.enable': False,
                    }],
                ),
                ComposableNode(
                    package='nitros_zc_bench',
                    plugin='nitros_zc_bench::NitrosViewer',
                    name='nitros_viewer',
                    parameters=[{
                        'topic': '/camera/camera/color/nitros_image',
                        'rate_hz': rate_hz,
                        'full_res': full_res,
                        'view_width': view_width,
                        'view_height': view_height,
                        'snapshot_path': snapshot_path,
                    }],
                ),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='1920x1080x30'),
        DeclareLaunchArgument('rate_hz', default_value='2.0'),
        DeclareLaunchArgument('full_res', default_value='false'),
        DeclareLaunchArgument('view_width', default_value='480'),
        DeclareLaunchArgument('view_height', default_value='270'),
        DeclareLaunchArgument('snapshot_path', default_value=''),
        OpaqueFunction(function=_container),
    ])

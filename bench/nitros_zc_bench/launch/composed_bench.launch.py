"""Camera node and one consumer composed into a single process.

This is the configuration where NITROS actually hands over a GPU pointer. Across a process
boundary rclcpp has to run the NitrosImage type adapter, whose convert_to_ros_message does a
device->host cudaMemcpy2D (and convert_to_custom copies back host->device on the subscriber
side) - so a separately launched consumer is not really zero-copy at all.

Note what is deliberately NOT set here: `use_intra_process_comms`. NITROS turns intra-process
comms on itself, per data publisher and subscriber (IntraProcessSetting::Enable in
nitros_publisher.cpp / nitros_subscriber.cpp). Setting it at node level instead applies it to the
`negotiated` package's transient-local negotiation topics too, and rclcpp rejects that combination
with "intraprocess communication allowed only with volatile durability". NVIDIA's own Isaac ROS
launch files compose NITROS nodes without the flag for the same reason.

    ros2 launch nitros_zc_bench composed_bench.launch.py consumer:=nitros duration_s:=30.0
    ros2 launch nitros_zc_bench composed_bench.launch.py consumer:=cpu    duration_s:=30.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _container(context, *args, **kwargs):
    consumer = LaunchConfiguration('consumer').perform(context)
    warmup_s = float(LaunchConfiguration('warmup_s').perform(context))
    duration_s = float(LaunchConfiguration('duration_s').perform(context))
    profile = LaunchConfiguration('profile').perform(context)
    csv_path = LaunchConfiguration('csv_path').perform(context)
    # Only meaningful with consumer:=cpu. NITROS cannot be combined with node-level intra-process
    # comms (see the module docstring); this exists to measure the best non-NITROS configuration,
    # where rclcpp passes the sensor_msgs/Image by pointer instead of serialising it.
    intra_process = LaunchConfiguration('intra_process').perform(context).lower() == 'true'
    extra = [{'use_intra_process_comms': True}] if intra_process else []

    consumer_plugin = {
        'nitros': 'nitros_zc_bench::NitrosConsumer',
        'cpu': 'nitros_zc_bench::CpuConsumer',
    }[consumer]
    consumer_params = {
        'warmup_s': warmup_s,
        'duration_s': duration_s,
        'expected_fps': 30.0,
        'csv_path': csv_path,
    }
    if consumer == 'nitros':
        # The wrapper's factory instantiates the node as /camera/camera, so its relative
        # topics land under that, not under /camera.
        consumer_params['topic'] = '/camera/camera/color/nitros_image'
    else:
        consumer_params['topic'] = '/camera/camera/color/image_raw'

    return [
        ComposableNodeContainer(
            name='bench_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            # One line per 100 frames from the publisher's own cost accounting.
            arguments=['--ros-args', '--log-level', 'NitrosImagePublisher:=debug'],
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name='camera',
                    extra_arguments=extra,
                    parameters=[{
                        'rgb_camera.color_profile': profile,
                        'rgb_camera.color_format': 'RGB8',
                        # Only turn NITROS publishing on when the NITROS consumer is the one
                        # running, so the plain-consumer run does not pay for an unused publisher.
                        'enable_color_nitros': consumer == 'nitros',
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
                    plugin=consumer_plugin,
                    name=f'{consumer}_consumer',
                    extra_arguments=extra,
                    parameters=[consumer_params],
                ),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('consumer', default_value='nitros', choices=['nitros', 'cpu']),
        DeclareLaunchArgument('warmup_s', default_value='5.0'),
        DeclareLaunchArgument('duration_s', default_value='30.0'),
        DeclareLaunchArgument('profile', default_value='1920x1080x30'),
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument('intra_process', default_value='false'),
        OpaqueFunction(function=_container),
    ])

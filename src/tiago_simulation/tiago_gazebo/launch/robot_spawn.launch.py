
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from dataclasses import dataclass
from launch_pal.arg_utils import LaunchArgumentsBase

from launch.actions import TimerAction

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    robot_name: DeclareLaunchArgument = DeclareLaunchArgument(
        name="robot_name", description="Gazebo model name"
    )


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    sim_dir = get_package_share_directory('tiago_bringup')

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-model",
            LaunchConfiguration("robot_name"),
            "-topic",
            "robot_description",
        ],
    )

    launch_description.add_action(gazebo_spawn_robot)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': os.path.join(
                    sim_dir, 'config/bridge', 'tiago_bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    launch_description.add_action(bridge)

    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/head_front_camera/image'])

    launch_description.add_action(camera_bridge_image)

    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/head_front_camera/depth_image'])

    launch_description.add_action(camera_bridge_depth)

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        output="screen",
        parameters = [
            {
                'frame_id': 'base_footprint',
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_muxed'),
            ('cmd_vel_out', '/mobile_base_controller/cmd_vel')
        ],
    )

    launch_description.add_action(twist_stamper)

    top_camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/top_camera/image_raw'])

   # launch_description.add_action(top_camera_bridge_image)

    right_camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/right_camera/image_raw'])

     #launch_description.add_action(right_camera_bridge_image)

    # ----------- Top View Camera -----------
    # Bridge for top view camera image
    top_view_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='top_view_camera_image_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/top_view_camera/image_raw']
    )
    launch_description.add_action(top_view_camera_bridge)

    # Delay the top view camera info bridge by 5 seconds
    top_view_camera_info_bridge = TimerAction(
        period=5.0,  # Delay to ensure the image stream is ready
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='top_view_camera_info_bridge',
                output='screen',
                arguments=['/top_view_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo']
            )
        ]
    )
    launch_description.add_action(top_view_camera_info_bridge)

    # ----------- Right-Side Camera -----------
    # Bridge for right-side camera image
    right_side_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='right_side_camera_image_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/right_side_camera/image_raw']
    )
    launch_description.add_action(right_side_camera_bridge)

    # Delay the right-side camera info bridge by 5 seconds
    right_side_camera_info_bridge = TimerAction(
        period=5.0,  # Delay to ensure the image stream is ready
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='right_side_camera_info_bridge',
                output='screen',
                arguments=['/right_side_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo']
            )
        ]
    )
    launch_description.add_action(right_side_camera_info_bridge)

    return

# Launch file that starts up the QCar2 nodes for EKF-based lidar/IMU/odometry
# scan-match localization, using localization_node.py against the
# ekf_scan_match RT model.
import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import (OnProcessExit, OnProcessStart)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def exit_qbot_platform_driver_interface_cb(context):

    subprocess.run('quarc_run' + ' -q -t tcpip://localhost:17000 qbot_platform_driver_physical', 
                   shell=True, 
                   capture_output=True)


def exit_ekf_scan_match_cb(context):

    subprocess.run('quarc_run' + ' -q -t tcpip://localhost:17000 ekf_scan_match', 
                   shell=True, 
                   capture_output=True)


def generate_launch_description():
    driver_model_rt_executable = PathJoinSubstitution([FindPackageShare('qbot_hardware'), 'rt_models', 'qbot_platform_driver_physical.rt-linux_qbot_platform'])
    driver_model_start = ExecuteProcess(
            cmd=['quarc_run', '-r -t tcpip://localhost:17000', driver_model_rt_executable, '-d %d -uri tcpip://%m:17001'],
            name='QBotPlatformDriverModelStart',
            shell=True
        )
    ekf_scan_match_rt_executable = PathJoinSubstitution([FindPackageShare('qbot_localization'), 'rt_models', 'ekf_scan_match.rt-linux_qbot_platform'])
    ekf_rt_model_start = ExecuteProcess(
            cmd=['quarc_run', '-r -t tcpip://localhost:17000', ekf_scan_match_rt_executable, '-d %d -uri tcpip://%m:17002'],
            name='EkfScanMatchModelStart',
            shell=True
        )

    # Declare launch arguments with default values
    declare_args = [
        DeclareLaunchArgument('odom_pub', default_value='false'),
        DeclareLaunchArgument('scan_tf_pub', default_value='false'),
        DeclareLaunchArgument('init_pose', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('speed_topic', default_value='/qbot_speed_feedback'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),
        DeclareLaunchArgument('rt_stream_uri', default_value='tcpip://localhost:18999'),
    ]

    qbot_platform_driver_node = Node(
            package='qbot_hardware',
            executable='qbot_platform_driver_interface',
            name='QBotPlatformDriver',
            parameters=[{'arm_robot': True}],
        )

    lidar_node = Node(
            package='qbot_hardware',
            executable='lidar',
            name='Lidar'
        )

    fixed_frame = Node(
            package='qbot_hardware',
            executable='fixed_lidar_frame',
            name='lidar_frame',
        )

    localization_node = Node(
            package='qbot_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            parameters=[{
                'odom_pub': LaunchConfiguration('odom_pub'),
                'tf_pub': LaunchConfiguration('scan_tf_pub'),
                'init_pose': LaunchConfiguration('init_pose'),
                'speed_topic': LaunchConfiguration('speed_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'rt_stream_uri': LaunchConfiguration('rt_stream_uri'),
                }]
            )
    start_after_driver_model = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_model_start,
            on_start=[
                LogInfo(msg="Driver RT model started. Waiting 2 seconds before starting ekf_scan_match RT model."),
                TimerAction(
                    period=2.0,
                    actions=[ekf_rt_model_start],
                ),
            ],
        )
    )
    start_after_rt_model = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_rt_model_start,
            on_start=[
                LogInfo(msg="EKF model started. Waiting 2 seconds before starting localization stack nodes."),
                TimerAction(
                    period=2.0,
                    actions=[
                             qbot_platform_driver_node,
                             lidar_node, 
                             fixed_frame, 
                             localization_node, 
                             ],
                ),
            ],
        )
    )

    stop_rt_model_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=qbot_platform_driver_node,
            on_exit=[
                OpaqueFunction(function=exit_qbot_platform_driver_interface_cb),
                LogInfo(msg="Driver node exited; stopping QUARC RT model."),
            ],
        )
    )

    stop_ekf_model_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_node,
            on_exit=[
                OpaqueFunction(function=exit_ekf_scan_match_cb),
                LogInfo(msg="localization_node exited; stopping ekf_scan_match RT model."),
            ],
        )
    )

    return LaunchDescription(
        declare_args + [
            driver_model_start,
            start_after_driver_model,
            start_after_rt_model,
            stop_rt_model_on_exit,
            stop_ekf_model_on_exit,
        ]
    )
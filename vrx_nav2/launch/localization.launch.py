from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rl_params_file = os.path.join(
        get_package_share_directory('vrx_nav2'),
        'config',
        'ekf_navsat_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_params',
            default_value='',
            description='Path to EKF parameters file'
        ),
        DeclareLaunchArgument(
            'navsat_params',
            default_value='',
            description='Path to NavSat transform parameters file'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[rl_params_file],
            remappings=[('/odometry/filtered', '/odometry/local')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[rl_params_file],
            remappings=[('/odometry/filtered', '/odometry/global')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[rl_params_file],
            remappings=[
                ('/imu/data', '/catamaran/sensors/imu/imu/data'),
                ('/gps/fix', '/catamaran/sensors/gps/gps/fix'),
                ('/odometry/filtered', '/odometry/global'),                # Already correct
                ('/odometry/gps', '/odometry/gps')              # Output topic
            ]
        )
    ])

#             remappings=[
#                 ('/imu/data', '/catamaran/sensors/imu/imu/data'),
#                 ('/gps/fix', '/catamaran/sensors/gps/gps/fix'),
#                 ('/odometry/filtered', '/odometry/gps')
#             ]
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ekf_params = LaunchConfiguration('ekf_params')
    navsat_params = LaunchConfiguration('navsat_params')

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
            name='ekf_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[('/odometry/filtered', '/odom')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_params],
            remappings=[
                ('/imu/data', '/catamaran/sensors/imu/imu/data'),
                ('/gps/fix', '/catamaran/sensors/gps/gps/fix'),
                ('/odometry/filtered', '/odom'),                # Already correct
                ('/odometry/gps', '/odometry/gps')              # Output topic
            ]
        )
    ])

#             remappings=[
#                 ('/imu/data', '/catamaran/sensors/imu/imu/data'),
#                 ('/gps/fix', '/catamaran/sensors/gps/gps/fix'),
#                 ('/odometry/filtered', '/odometry/gps')
#             ]
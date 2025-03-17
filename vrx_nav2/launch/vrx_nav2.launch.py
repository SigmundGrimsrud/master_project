from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def launch():
    launch_processes = []

    return launch_processes


def generate_launch_description():
    # Paths to configuration files
    nav2_params = os.path.join(
        get_package_share_directory('vrx_nav2'),
        'config',
        'nav2_params.yaml'
    )

    # Launch VRX
    # vrx_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('vrx_gz'),
    #         '/launch/simple.launch.py'
    #     ])
    # )
    vrx_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'vrx_gz', 'simple.launch.py'],
        output='screen')

    # Map server
    map_server = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p',
              'yaml_filename:=/home/sigmu/master/src/vrx/vrx_ros/src/solar_grid/solar_grid.yaml'],
        output='screen')
    
    map_activate = ExecuteProcess(
        cmd=[['sleep 4; ros2 lifecycle set /map_server configure; ros2 lifecycle set /map_server activate'
        ]],
        shell=True, output='screen')
    #; ros2 run vrx_nav2 set_datum.py

    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={'params_file': nav2_params}.items()
    )

    # Launch localization nodes
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('vrx_nav2'),
            '/launch/localization.launch.py'
        ])
    )

    # amcl_node = Node(  
    #     package='nav2_amcl',  
    #     executable='amcl',  
    #     name='amcl',  
    #     output='screen',  
    #     parameters=[{'use_sim_time': True}]  
    # )

    odometry_node = Node(
        package='vrx_nav2',
        executable='odometry_node.py',
        name='odometry_node',
        output='screen'
    )

    return LaunchDescription([
        vrx_launch,
        localization_launch,
        odometry_node,
        nav2_launch,
        map_server,
        map_activate
        # amcl_node
    ])
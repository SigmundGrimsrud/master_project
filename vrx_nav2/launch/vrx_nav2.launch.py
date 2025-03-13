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
    ekf_params = os.path.join(
        get_package_share_directory('vrx_nav2'),
        'config',
        'ekf_localization.yaml'
    )
    navsat_params = os.path.join(
        get_package_share_directory('vrx_nav2'),
        'config',
        'navsat_transform.yaml'
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
    
    # Does not work, must be launched in terminal manually as of now
    # map_activate = ExecuteProcess(
    #     cmd=['sleep', '60',';', 'ros2', 'lifecycle', 'set', '/map_server', 'configure;',
    #          'sleep', '1', ';', 'ros2', 'lifecycle', 'set', '/map_server', 'activate'],
    #     output='screen')
    map_activate = ExecuteProcess(
        cmd=[['sleep 10; ros2 lifecycle set /map_server configure; ros2 lifecycle set /map_server activate; ros2 run vrx_nav2 set_datum.py'
        ]],
        shell=True, output='screen')

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
        ]),
        launch_arguments={
            'ekf_params': ekf_params,
            'navsat_params': navsat_params
        }.items()
    )

    amcl_node = Node(  
        package='nav2_amcl',  
        executable='amcl',  
        name='amcl',  
        output='screen',  
        parameters=[{'use_sim_time': True}]  
    )

    odometry_node = Node(
        package='vrx_nav2',
        executable='odometry_node.py',
        name='odometry_node',
        output='screen'
    )

    return LaunchDescription([
        vrx_launch,
        nav2_launch,
        localization_launch,
        odometry_node,
        map_server,
        map_activate,
        amcl_node
    ])
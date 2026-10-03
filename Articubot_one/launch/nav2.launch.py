import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    articubot_dir = get_package_share_directory('articubot_one')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Declare CLI Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(articubot_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(articubot_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # 1. Localization Stack (Map Server + AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml_file,            # Pass CLI map argument here
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 2. Navigation Stack (Planner, Controller, BT Navigator, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        localization_launch,
        navigation_launch,
    ])
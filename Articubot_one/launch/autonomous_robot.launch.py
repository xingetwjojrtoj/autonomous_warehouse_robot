import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_articubot_one = get_package_share_directory('articubot_one')

    # Include simulation launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_articubot_one, 'launch', '2wheel_launch_sim.launch.py'
            )
        )
    )

    # Include Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_articubot_one, 'launch', 'nav2.launch.py')
        )
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch,
    ])
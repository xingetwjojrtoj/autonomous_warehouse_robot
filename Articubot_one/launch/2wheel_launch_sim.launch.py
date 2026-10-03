import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','2wheel_rsp.launch.py'
                )]), launch_arguments={'use_sim_time':'false','use_ros2_control':'true'}.items()
    )


    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    #          )
    
    

    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot1'],
    #                     output='screen')
    
    diff_drive_spawner = Node(package='controller_manager', executable='spawner',
                        arguments=['diff_cont'])  #don,t know why there is no .py in mine spawner , its there in articubot one , but ie its ntot working
    

    joint_broad_spawner = Node(package='controller_manager', executable='spawner',
                        arguments=['joint_broad'])
    


    # for GPS localization 

    ekf_config = os.path.join(
        get_package_share_directory('articubot_one'), 'config', 'ekf_gps1.yaml'
    )

    navsat_transform_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
            remappings=[
                # ('imu/data', 'imu/mag_heading'),
                ('imu/data','imu/data_raw'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ]
        )

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[ekf_config, {'use_sim_time':False}],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            ('imu/data','imu/data_raw')
        ]
    )

    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[ekf_config, {'use_sim_time':False}],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            ('imu/data','imu/data_raw')
        ]
    )


    # # heading_publisher_node = Node(
    # #     package='articubot_one',
    # #     executable='heading_publisher.py',
    # #     name='heading_publisher',
    # #     output='screen',
    # #     # parameters=[{'use_sim_time':'true'}]
    # #     parameters=[{'use_sim_time': ParameterValue(True, value_type=bool)}]
    # # )



    imu_serial_bridge_node = Node(
        package='articubot_one',
        executable='IMU_node.py',
        name='imu_serial_bridge',
        output='screen',
        # parameters=[{'use_sim_time':False,
                    #  'port': '/dev/ttyUSB0'}]
    )
    




    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # output='screen',
        arguments=['0.1', '0.0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        )
    

    scan_filter_node = Node(
        package='articubot_one',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 6. Custom GPS/GNSS Driver Node
    gps_driver_node = Node(
        package='articubot_one',
        executable='gps_node.py', # Make sure this matches your file name exactly
        name='gps_gnss_driver_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )


    # Locate the rplidar_ros share directory
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    
    # Include the RPLIDAR C1 launch file
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_dir, 'launch', 'rplidar_c1_launch.py')
        )
    )
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )
    
    arduino_node=Node(
        package='articubot_one',
        executable='arduino_node.py',
        name='arduino_node',
        output='screen',
        # parameters=[{
        # 'port': '/dev/ttyCH341USB0',
        # 'baudrate': 115200,
        # }]
        )


    # Odom_publisher=Node(
    #     package='articubot_one',
    #     executable='wheel_odom_publisher.py',
    #     name='wheel_odom_publisher',
    #     output='screen',
    #     )

    
    gps_nav2_bridge_node = Node(
        package='articubot_one',
        executable='gps_nav2_bridge.py',  # Ensure script is executable & in CMakeLists.txt / setup.py
        name='gps_nav2_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        diff_drive_spawner,
        joint_broad_spawner,
        navsat_transform_node,
        local_ekf_node,
        global_ekf_node,
        gps_nav2_bridge_node,
        # joint_state_publisher_gui,
        # Odom_publisher,
        rplidar_launch,
        scan_filter_node,
        gps_driver_node,
        imu_static_tf,
        imu_serial_bridge_node,
        arduino_node
    ])
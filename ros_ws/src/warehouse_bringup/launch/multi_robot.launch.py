import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ====================================================================
    # 1. PACKAGE AND LAUNCH FILE PATHS
    # ====================================================================
    gazebo_pkg = get_package_share_directory('warehouse_gazebo')
    robot_pkg = get_package_share_directory('warehouse_robot')

    gazebo_launch_file = os.path.join(gazebo_pkg, 'launch', 'launch_warehouse.py')
    spawn_launch_file = os.path.join(robot_pkg, 'launch', 'launch_robot.launch.py')
    nav2_launch_file = os.path.join(robot_pkg, 'launch', 'navigation.launch.py')

    # ====================================================================
    # 2. GAZEBO PHYSICAL COORDINATES AND INDICES
    # ====================================================================
    robots = {
        'robot1': {'x': '21.19', 'y': '-5.81', 'yaw': '1.51',  'index': 0},
      # 'robot2': {'x': '21.18', 'y': '7.77',  'yaw': '3.13',  'index': 1},
      # 'robot3': {'x': '-6.87', 'y': '7.76',  'yaw': '-1.46', 'index': 2}
    }

    # ====================================================================
    # 3. LAUNCH ACTIONS DEFINITION
    # ====================================================================
    ld = LaunchDescription()

    # Initialize simulation environment
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        )
    )

    # ====================================================================
    # 4. SEQUENTIAL ROBOT SPAWN WITH TIMER DELAY
    # ====================================================================
    delay_time = 0.0  # Initial spawn delay (seconds)

    for robot_name, coords in robots.items():
        
        # Group actions for the current robot
        robot_actions = []

        # Spawn robot in Gazebo
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch_file),
                launch_arguments={
                    'robot_name': robot_name,
                    'x': coords['x'],
                    'y': coords['y'],
                    'yaw': coords['yaw']
                }.items()
            )
        )

        # Initialize Nav2 stack
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'robot_name': robot_name,
                    'use_sim_time': 'True' 
                }.items()
            )
        )

        # YOLO Detector node (Disabled)
        # robot_actions.append(
        #    Node(
        #        package='warehouse_robot',
        #        executable='yolo_detector',
        #        namespace=robot_name,
        #        output='screen'
        #    )
        # )

        # Camera Tracker node (Disabled)
        # robot_actions.append(
        #    Node(
        #        package='warehouse_robot',
        #        executable='camera_tracker',
        #        namespace=robot_name,
        #        output='screen'
        #    )
        # )

        # Initialize Robot Manager
        robot_actions.append(
            Node(
                package='warehouse_robot',
                executable='robot_manager',
                namespace=robot_name,
                parameters=[{
                    'start_wp_index': coords['index'],
                    'use_sim_time': True
                }],
                remappings=[
                    ('/tf', f'/{robot_name}/tf'),
                    ('/tf_static', f'/{robot_name}/tf_static'),
                    ('/clock', '/clock')
                ],
                output='screen'
            )
        )

        # Wrap robot actions in a TimerAction for staggered initialization
        ld.add_action(
            TimerAction(
                period=delay_time,
                actions=robot_actions
            )
        )

        # Increment delay for the next robot iteration
        delay_time += 20.0

    # ====================================================================
    # 5. GLOBAL FLEET MANAGER INITIALIZATION
    # ====================================================================
    global_manager_action = Node(
        package='fleet_manager',
        executable='global_manager',
        name='global_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    ld.add_action(
        TimerAction(
            period=50.0, 
            actions=[global_manager_action]
        )
    )

    # ====================================================================
    # 6. INTRUDER ACTOR: SPAWN TIMING
    # ====================================================================
    ladro_sdf_path = os.path.join(gazebo_pkg, 'worlds', 'ladro.sdf')

    spawn_ladro_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', ladro_sdf_path,
            '-name', 'attore_ladro'
        ],
        output='screen'
    )

    # Actor spawn trigger (60s total delay, 10s post-Global Manager)
    ld.add_action(
        TimerAction(
            period=60.0,
            actions=[spawn_ladro_node]
        )
    )

    # Note: Actor despawn is handled internally via SDF trajectory.

    return ld
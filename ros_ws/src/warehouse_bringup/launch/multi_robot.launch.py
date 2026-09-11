import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # ====================================================================
    # 1. PACKAGE AND RESOURCE PATH RESOLUTION
    # ====================================================================
    gazebo_pkg = get_package_share_directory('warehouse_gazebo')
    robot_pkg = get_package_share_directory('warehouse_robot')

    gazebo_launch_file = os.path.join(gazebo_pkg, 'launch', 'launch_warehouse.py')
    spawn_launch_file = os.path.join(robot_pkg, 'launch', 'launch_robot.launch.py')
    nav2_launch_file = os.path.join(robot_pkg, 'launch', 'navigation.launch.py')

    # ====================================================================
    # 2. AGENT INITIALIZATION PARAMETERS
    # ====================================================================
    robots = {
        'robot1': {'x': '21.19', 'y': '-5.81', 'yaw': '1.51',  'index': 0},
        'robot2': {'x': '21.18', 'y': '7.77',  'yaw': '3.13',  'index': 1},
        'robot3': {'x': '-6.87', 'y': '7.76',  'yaw': '-1.46', 'index': 2}
    }

    ld = LaunchDescription()

    # ====================================================================
    # 3. ENVIRONMENT AND CORE SERVICES INITIALIZATION
    # ====================================================================
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        )
    )

    tf_merger_node = Node(
        package='warehouse_robot',
        executable='tf_merger',
        name='tf_merger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(tf_merger_node)

    # ====================================================================
    # 4. SEQUENTIAL AGENT DEPLOYMENT STRATEGY
    # ====================================================================
    # Staggered initialization mitigates CPU burst loads and thermal throttling.
    spawn_delay = 8.0   
    nav2_delay = 45.0   

    for robot_name, coords in robots.items():
        
        # Instantiate physical robot model in the simulation
        spawn_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_file),
            launch_arguments={
                'robot_name': robot_name,
                'x': coords['x'],
                'y': coords['y'],
                'yaw': coords['yaw']
            }.items()
        )
        
        ld.add_action(
            TimerAction(period=spawn_delay, actions=[spawn_action])
        )

        # Initialize navigation stack and decentralized local manager
        nav2_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': 'True' 
            }.items()
        )

        manager_action = Node(
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

        ld.add_action(
            TimerAction(period=nav2_delay, actions=[nav2_action, manager_action])
        )

        # Increment temporal offsets for subsequent agents
        spawn_delay += 10.0  
        nav2_delay += 20.0    

    # ====================================================================
    # 5. CENTRALIZED FLEET MANAGER AND LIFECYCLE EVENT HANDLING
    # ====================================================================
    global_manager_action = Node(
        package='fleet_manager',
        executable='global_manager',
        name='global_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Register an event handler for graceful simulation termination 
    # triggered by the global manager's successful execution exit code.
    shutdown_on_capture = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=global_manager_action,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    ld.add_action(
        TimerAction(period=85.0, actions=[global_manager_action, shutdown_on_capture])
    )

    # ====================================================================
    # 6. DYNAMIC ACTOR (INTRUDER) INJECTION
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

    ld.add_action(
        TimerAction(period=135.0, actions=[spawn_ladro_node])
    )

    return ld
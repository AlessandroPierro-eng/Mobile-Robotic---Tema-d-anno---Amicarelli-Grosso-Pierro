import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ====================================================================
    # 1. PERCORSI DEI PACCHETTI E DEI LAUNCH FILE
    # ====================================================================
    gazebo_pkg = get_package_share_directory('warehouse_gazebo')
    robot_pkg = get_package_share_directory('warehouse_robot')

    gazebo_launch_file = os.path.join(gazebo_pkg, 'launch', 'launch_warehouse.py')
    spawn_launch_file = os.path.join(robot_pkg, 'launch', 'launch_robot.launch.py')
    nav2_launch_file = os.path.join(robot_pkg, 'launch', 'navigation.launch.py')

    # ====================================================================
    # 2. DIZIONARIO DELLE COORDINATE FISICHE (GAZEBO) CON INDICI
    # ====================================================================
    robots = {
        'robot1': {'x': '21.19', 'y': '-5.81', 'yaw': '1.51',  'index': 0},
      #  'robot2': {'x': '21.18', 'y': '7.77',  'yaw': '3.13',  'index': 1},
     #   'robot3': {'x': '-6.87', 'y': '7.76',  'yaw': '-1.46', 'index': 2}
    }

    # ====================================================================
    # 3. LISTA DELLE AZIONI DI LANCIO
    # ====================================================================
    ld = LaunchDescription()

    # Avvio dell'ambiente (Terminale 1)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        )
    )

    # ====================================================================
    # 4. CICLO DI CREAZIONE CON RITARDO (TIMER ACTION)
    # ====================================================================
    delay_time = 0.0  # Ritardo iniziale di 0 secondi

    for robot_name, coords in robots.items():
        
        # Raggruppiamo tutte le azioni di QUESTO specifico robot
        robot_actions = []

        # Spawn in Gazebo
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

        # Avvio Nav2
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'robot_name': robot_name,
                    'use_sim_time': 'True' 
                }.items()
            )
        )

        # Avvio Yolo Detector (DISABILITATO)
        # robot_actions.append(
        #    Node(
        #        package='warehouse_robot',
        #        executable='yolo_detector',
        #        namespace=robot_name,
        #        output='screen'
        #    )
        # )

        # Avvio Camera Tracker (DISABILITATO)
        # robot_actions.append(
        #    Node(
        #        package='warehouse_robot',
        #        executable='camera_tracker',
        #        namespace=robot_name,
        #        output='screen'
        #    )
        # )

        # Avvio Robot Manager
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

        # Avvolgiamo le azioni del robot in un TimerAction con il ritardo attuale
        ld.add_action(
            TimerAction(
                period=delay_time,
                actions=robot_actions
            )
        )

        # Aumentiamo il ritardo di 20 secondi per il prossimo robot nel ciclo
        delay_time += 20.0

    # ====================================================================
    # 5. AVVIO DEL GLOBAL FLEET MANAGER
    # ====================================================================
    # Alla fine del ciclo, delay_time è diventato 60.0.
    # Vogliamo che il manager parta 10 secondi DOPO il Robot 3 (che è partito a 40.0).
    # Quindi il timer per il manager sarà a 50.0 secondi.
    
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

    return ld
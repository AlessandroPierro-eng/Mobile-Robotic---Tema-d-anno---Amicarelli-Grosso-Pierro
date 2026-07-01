import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_warehouse = get_package_share_directory('warehouse_gazebo')
    # Trova il pacchetto originale del TurtleBot
    pkg_turtlebot = get_package_share_directory('turtlebot4_description')

    # 1. GESTIONE DEL MONDO 3D
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tugbot_depot.sdf', # Lancio il tag depot creato nella cartella worlds
        description='Nome del file del mondo da caricare'
    )

    world_path = PathJoinSubstitution([
        pkg_warehouse,
        'worlds',
        LaunchConfiguration('world')
    ])

    # 2. AVVIO DI GAZEBO HARMONIC
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Il parametro '-r' fa partire subito la simulazione
        # Il parametro '-v 4' ci dà tutti i log nel terminale
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items() 
    )

    # 3. LETTURA DEL MODELLO DEL ROBOT (XACRO)
    # Punta al file originale che hai modificato con la telecamera motorizzata
    xacro_file = os.path.join(pkg_turtlebot, 'urdf', 'lite', 'turtlebot4.urdf.xacro')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )

    # 4. SPAWN DEL ROBOT NEL MAGAZZINO
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot4',
            '-topic', 'robot_description',
            '-x', '7.0',
            '-y', '0.0',
            '-z', '0.1'  # Partenza a 10 cm da terra per evitare rimbalzi
        ],
        output='screen'
    )

    # 5. RESTITUZIONE DI TUTTE LE ISTRUZIONI A ROS 2
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_publisher, # Aggiunto il nodo del modello
        spawn_entity           # Aggiunto il nodo di spawn
    ])
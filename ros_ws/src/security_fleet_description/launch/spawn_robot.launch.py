import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Trova la cartella del nostro pacchetto
    pkg_share = get_package_share_directory('security_fleet_description')
    
    # Percorso esatto al file Xacro che hai creato
    xacro_file = os.path.join(pkg_share, 'urdf', 'security_turtlebot.urdf.xacro')

    # Nodo 1: Robot State Publisher (Il "cervello" che calcola dove sono i pezzi del robot)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )

    # Nodo 2: Il teletrasporto dentro Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'security_turtlebot',
            '-topic', 'robot_description', # Prende il modello dal nodo sopra
            '-x', '7.0',  # Coordinata X nel magazzino
            '-y', '0.0',  # Coordinata Y nel magazzino
            '-z', '0.1'   # Lo facciamo nascere leggermente sollevato per non incastrarlo nel pavimento
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity
    ])
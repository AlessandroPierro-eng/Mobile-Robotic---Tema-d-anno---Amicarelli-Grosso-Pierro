import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Recupera i percorsi dei pacchetti installati
    pkg_warehouse_robot = get_package_share_directory('warehouse_robot')
    pkg_warehouse_gazebo = get_package_share_directory('warehouse_gazebo') # <-- AGGIUNTO
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. Definisce i percorsi assoluti
    # <-- LA MAPPA ORA LA PRENDE DA WAREHOUSE_GAZEBO -->
    map_file = os.path.join(pkg_warehouse_gazebo, 'map', 'mappa_magazzino.yaml') 
    
    params_file = os.path.join(pkg_warehouse_robot, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_warehouse_robot, 'config', 'nav_config.rviz')

    # 3. Parametro fondamentale per il simulatore
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 4. Chiama il motore di Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 5. Lancia RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. Costruisce e restituisce l'elenco delle azioni
    ld = LaunchDescription()
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     # Trova i percorsi ai pacchetti nel computer
#     pkg_warehouse_gazebo = get_package_share_directory('warehouse_gazebo')
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

#     # Percorso esatto al tuo file del mondo (assicurati che si chiami warehouse.world)
#     world_file = os.path.join(pkg_warehouse_gazebo, 'worlds', 'warehouse.world')

#     # Variabile d'ambiente FONDAMENTALE: dice a Gazebo dove andare a pescare i muri e gli scaffali 3D
#     set_env_vars_resources = AppendEnvironmentVariable(
#         'GZ_SIM_RESOURCE_PATH',
#         os.path.join(pkg_warehouse_gazebo, 'models') + ':' + pkg_warehouse_gazebo
#     )

#     # Chiama il comando di avvio ufficiale di Gazebo Harmonic
#     # -r = avvia la simulazione (run)
#     # -v 4 = mostra eventuali errori nel terminale (verbosity)
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
#         ),
#         launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
#     )

#     return LaunchDescription([
#         set_env_vars_resources,
#         gz_sim
#     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Trova la cartella di installazione del tuo pacchetto (usa il nome esatto del tuo pacchetto)
    pkg_warehouse = get_package_share_directory('warehouse_gazebo')

    # ---------------------------------------------------------
    # 1. IL TRUCCO PER LE TEXTURE (GZ_SIM_RESOURCE_PATH)
    # Diciamo a Gazebo che in questa cartella ci sono file utili
    # ---------------------------------------------------------
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_warehouse
    )

    # ---------------------------------------------------------
    # 2. GESTIONE DEL MONDO 3D
    # Permette di scegliere il file .sdf, di default carica il magazzino
    # ---------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse.sdf', # <--- ATTENZIONE: Metti il nome esatto del tuo file .sdf!
        description='Nome del file SDF del mondo da caricare'
    )

    world_path = PathJoinSubstitution([
        pkg_warehouse,
        'worlds',
        LaunchConfiguration('world')
    ])

    # ---------------------------------------------------------
    # 3. AVVIO DI GAZEBO HARMONIC
    # ---------------------------------------------------------
    gazebo_launch = IncludeLaunchArgument(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Il parametro '-r' serve per far partire subito la simulazione senza dover premere Play
        launch_arguments={'gz_args': ['-r ', world_path]}.items() 
    )

    # ---------------------------------------------------------
    # 4. RESTITUZIONE DI TUTTE LE ISTRUZIONI A ROS 2
    # ---------------------------------------------------------
    return LaunchDescription([
        set_env_vars,
        world_arg,
        gazebo_launch
    ])
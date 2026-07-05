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

    # 5. BRIDGE
    # Ponte ROS <-> Gazebo per Motori e Sensori (crea il collegamento tra motori con LiDAR, telecamera ed IMU)
    gazebo_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # --- MOTORI (Bidirezionale) ---
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/bracket_vel@std_msgs/msg/Float64@gz.msgs.Double',
            
            # --- SENSORI (Da Gazebo a ROS2) ---
            # 1. Telecamera (Immagini standard)
            '/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # 2. Telecamera di Profondità 
            '/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # 3. IMU (Accelerometro/Giroscopio)
            '/world/world_demo/model/turtlebot4/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # 4. Lidar (Scansione Laser 2D)
            '/world/world_demo/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # 5. Odometria (Movimento stimato dalle ruote pubblicato direttamente sulla root)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # 6. Transformazioni (TF - Posizione dinamica del robot nel mondo)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        # Configurazione di nuovi nomi (più corti)
        remappings=[
            ('/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image', '/camera/image_raw'),
            ('/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image', '/camera/depth/image_raw'),
            ('/world/world_demo/model/turtlebot4/link/imu_link/sensor/imu/imu', '/imu'),
            ('/world/world_demo/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan', '/scan'),
        ],
        output='screen'
    )

    # 6. RESTITUZIONE DI TUTTE LE ISTRUZIONI A ROS 2
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_publisher, # Aggiunto il nodo del modello
        spawn_entity,           # Aggiunto il nodo di spawn
        gazebo_bridges         # Aggiunto il ponte
    ])
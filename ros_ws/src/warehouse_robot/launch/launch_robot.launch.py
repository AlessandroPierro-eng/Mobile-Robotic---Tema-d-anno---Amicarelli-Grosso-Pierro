import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_turtlebot = get_package_share_directory('turtlebot4_description')
    xacro_file = os.path.join(pkg_turtlebot, 'urdf', 'lite', 'turtlebot4.urdf.xacro')

    # ==========================================
    # 1. DEFINIZIONE DEGLI ARGOMENTI DI AVVIO
    # ==========================================
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    yaw_pose = LaunchConfiguration('yaw') # <-- AGGIUNTO

    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='robot1', description='Namespace del robot')
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Posizione X in Gazebo')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Posizione Y in Gazebo')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Rotazione Yaw in Gazebo') # <-- AGGIUNTO

    # ==========================================
    # 2. NODI DI STATO (URDF e Giunti)
    # ==========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' namespace:=', robot_name]), 
                value_type=str
            ),
            'use_sim_time': True,
            'frame_prefix': [robot_name, '/']
        }],
        remappings=[
            ('/tf', ['/', robot_name, '/tf']),
            ('/tf_static', ['/', robot_name, '/tf_static'])
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            'source_list': ['bracket_joint_state'] 
        }]
    )

    # ==========================================
    # 3. SPAWN IN GAZEBO
    # ==========================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', [robot_name, '/robot_description'],
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1',
            '-Y', yaw_pose # <-- AGGIUNTO (Nota la 'Y' maiuscola richiesta da Gazebo per lo Yaw)
        ],
        output='screen'
    )

    # ==========================================
    # 4. BRIDGE ROS-GAZEBO (NOMI DINAMICI)
    # ==========================================
    gazebo_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_name,
        parameters=[{'use_sim_time': True}],
        arguments=[
            ['/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            ['/', robot_name, '/bracket_vel@std_msgs/msg/Float64@gz.msgs.Double'],
            
            # Sensori
            ['/world/world_demo/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            ['/world/world_demo/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
            ['/world/world_demo/model/', robot_name, '/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            ['/world/world_demo/model/', robot_name, '/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            
            # Odometria 
            ['/', robot_name, '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            ['/', robot_name, '/bracket_joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
            
            # Globali
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/world_demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            (['/world/world_demo/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'], 'camera/image_raw'),
            (['/world/world_demo/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image'], 'camera/depth/image_raw'),
            (['/world/world_demo/model/', robot_name, '/link/imu_link/sensor/imu/imu'], 'imu'),
            (['/world/world_demo/model/', robot_name, '/link/rplidar_link/sensor/rplidar/scan'], 'scan'),
            ('/world/world_demo/clock', '/clock'),
            ('/tf', ['/', robot_name, '/tf']),
            ('/tf_static', ['/', robot_name, '/tf_static'])
        ],
        output='screen'
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        arguments=[
            '0', '0', '0', '0', '0', '0', 
            [robot_name, '/rplidar_link'], 
            [robot_name, '/rplidar_link/rplidar']
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', ['/', robot_name, '/tf']),
            ('/tf_static', ['/', robot_name, '/tf_static'])
        ]
    )

    # ==========================================
    # COSTRUZIONE DEL LAUNCH
    # ==========================================
    ld = LaunchDescription()

    ld.add_action(declare_robot_name)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_yaw)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_bridges)
    ld.add_action(lidar_static_tf)

    return ld
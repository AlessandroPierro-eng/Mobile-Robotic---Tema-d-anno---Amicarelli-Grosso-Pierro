import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_turtlebot = get_package_share_directory('turtlebot4_description')

    # URDF/XACRO parsing and Robot State Publisher initialization
    xacro_file = os.path.join(pkg_turtlebot, 'urdf', 'lite', 'turtlebot4.urdf.xacro')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Gazebo entity spawning
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot4',
            '-topic', 'robot_description',
            '-x', '7.0',
            '-y', '0.0',
            '-z', '0.1'  # Initial Z offset for collision prevention
        ],
        output='screen'
    )

    # ROS 2 - Gazebo transport bridge configuration
    gazebo_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/bracket_vel@std_msgs/msg/Float64@gz.msgs.Double',
            '/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/turtlebot4/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/world_demo/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/world_demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image', '/camera/image_raw'),
            ('/world/world_demo/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image', '/camera/depth/image_raw'),
            ('/world/world_demo/model/turtlebot4/link/imu_link/sensor/imu/imu', '/imu'),
            ('/world/world_demo/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan', '/scan'),
            ('/world/world_demo/clock', '/clock'),
        ],
        output='screen'
    )

    # Static TF broadcasting for LiDAR frame alignment
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'rplidar_link', 'turtlebot4/rplidar_link/rplidar'],
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_bridges)
    ld.add_action(lidar_static_tf)

    return ld
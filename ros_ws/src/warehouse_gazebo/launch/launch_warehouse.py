import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_warehouse = get_package_share_directory('warehouse_gazebo')

    # Hardware acceleration check
    has_gpu = False
    try:
        subprocess.check_output(['nvidia-smi'])
        has_gpu = True
        print("[INFO] GPU hardware acceleration: ENABLED.")
    except Exception:
        has_gpu = False
        print("[INFO] GPU hardware acceleration: DISABLED. Software rendering fallback.")

    # Simulation environment arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tugbot_depot.sdf',
        description='Target SDF world file for simulation'
    )

    world_path = PathJoinSubstitution([
        pkg_warehouse,
        'worlds',
        LaunchConfiguration('world')
    ])

    # Gazebo Harmonic initialization
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items() 
    )

    ld = LaunchDescription()

    # Environment variables injection for CPU rendering fallback
    if not has_gpu:
        ld.add_action(SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'))
        ld.add_action(SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3'))

    ld.add_action(world_arg)
    ld.add_action(gazebo_launch)

    return ld
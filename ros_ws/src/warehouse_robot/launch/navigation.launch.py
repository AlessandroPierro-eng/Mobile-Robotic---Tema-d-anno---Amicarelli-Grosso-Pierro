import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from nav2_common.launch import ReplaceString

# ====================================================================
# INITIAL POSES DICTIONARY (NAV2 / AMCL)
# ====================================================================
INITIAL_POSES = {
    'robot1': {'x': '7.087', 'y': '-8.953', 'yaw': '1.551'},
    'robot2': {'x': '8.313', 'y': '4.530', 'yaw': '3.062'},
    'robot3': {'x': '-19.794', 'y': '6.758', 'yaw': '-1.516'}
}

def launch_setup(context, *args, **kwargs):
    # Extract the actual string value from LaunchConfiguration
    robot_name_str = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Retrieve initial coordinates from the dictionary based on robot name
    pose = INITIAL_POSES.get(robot_name_str, INITIAL_POSES['robot1'])

    pkg_warehouse_robot = get_package_share_directory('warehouse_robot')
    pkg_warehouse_gazebo = get_package_share_directory('warehouse_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_warehouse_gazebo, 'map', 'mappa_magazzino.yaml') 
    params_file = os.path.join(pkg_warehouse_robot, 'config', 'nav2_params.yaml')

    # ====================================================================
    # PARAMETER STRING REPLACEMENT (Namespace & Initial Coordinates)
    # ====================================================================
    configured_params = ReplaceString(
        source_file=params_file,
        replacements={
            '<robot_namespace>': robot_name_str,
            '<init_x>': pose['x'],
            '<init_y>': pose['y'],
            '<init_yaw>': pose['yaw']
        }
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'true',
            'use_namespace': 'true',
            'namespace': robot_name_str,
        }.items()
    )

    return [nav2_launch]


def generate_launch_description():
    # ====================================================================
    # LAUNCH ARGUMENT DECLARATIONS
    # ====================================================================
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Robot namespace'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    force_sim_time = SetParameter(name='use_sim_time', value=True)

    ld = LaunchDescription()
    
    # 1. Add argument declarations
    ld.add_action(declare_robot_name)
    ld.add_action(declare_use_sim_time)
    
    # 2. Force simulation time globally
    ld.add_action(force_sim_time)
    
    # 3. Execute Python setup logic
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
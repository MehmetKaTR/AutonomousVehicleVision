
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = FindPackageShare(package='simulation_2025').find('simulation_2025')

    gz_sim_resource_path = os.path.join(pkg_share, 'models')
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_sim_resource_path
    
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': '-r TeknofestWORLDV2.sdf'
            }.items(),
    )

    robot_spawner = TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'robot_spawner.launch.py')
                        )
                    )
                ]
    )

    material_color_bridge= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/material_color@ros_gz_interfaces/msg/MaterialColor@gz.msgs.MaterialColor'
        ]
    )

    clock_bridge= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        remappings=[
                ('/world/default/clock', '/clock')
            ]
    )

    traffic_light= Node(
        package='simulation_2025',
        executable='traffic_light.py',
        name='traffic_light'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
   
    # Add any actions
    ld.add_action(gz_sim)
    ld.add_action(material_color_bridge)
    ld.add_action(clock_bridge)
    ld.add_action(traffic_light)
    ld.add_action(robot_spawner)

    return ld

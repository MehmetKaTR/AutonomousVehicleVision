
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_share = FindPackageShare(package='simulation_2025').find('simulation_2025')

    model_path=os.path.join(pkg_share,'urdf/otagg_car.urdf.xacro')
    gz_sim_resource_path = os.path.join(pkg_share, 'models')
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_sim_resource_path

    urdf_model = LaunchConfiguration('urdf_model')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=model_path, 
        description='Absolute path to robot urdf file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_model])}]
    )

    entity_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world','default',
            '-topic','robot_description',
            '-x','2.0',
            '-y','1.5',
            '-Y','3.14'
            ]
    )

    ros_gz_bridge_node= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/mag@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
            '/depth/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu/data_raw@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom_raw@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ]
    )

    compressed_camera_publisher = Node(
        package='image_transport',
        executable='republish',
        arguments=[
            'raw',
            'compressed',
            ],
        remappings=[
            ('/in', '/camera'),
            ('/out/compressed', '/camera_compressed'),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
   
    # Add any actions
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(entity_spawner)
    # ld.add_action(tf_bridge)
    ld.add_action(ros_gz_bridge_node)
    ld.add_action(compressed_camera_publisher)

    return ld

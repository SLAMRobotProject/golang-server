import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Get the full path to your .lua configuration file
    config_file_dir = os.path.abspath(os.path.dirname(__file__))
    config_file_name = 'cartographer_config.lua'
    config_file_path = os.path.join(config_file_dir, config_file_name)

    # 2. Get the full path to the URDF file
    # Assuming the URDF is in the same directory as the launch file
    urdf_file_name = 'nrf52480dk_robot.urdf'
    urdf_path = os.path.join(config_file_dir, urdf_file_name)
    
    # Check if URDF exists (optional, but good practice)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # 3. Robot State Publisher Node (REQUIRED FOR RViz visualization)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )
    
    # Existing Static TF Publishers
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # x, y, z, yaw, pitch, roll: 0, 0, 0.1, 0, 0, 0
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        # x, y, z, yaw, pitch, roll: 0, 0, 0, 0, 0, 0
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )
    
    # Existing Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_file_dir,
            '-configuration_basename', config_file_name
        ]
    )

    # Existing Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.03']
    )

    return LaunchDescription([
        # Add the robot state publisher to the launch description
        robot_state_publisher_node, 
        
        static_tf_laser,
        static_tf_imu,
        
        cartographer_node,
        occupancy_grid_node
    ])
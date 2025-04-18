import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def check_file_exists(file_path, description):
    
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Cannot find {description} at: {file_path}")

def generate_launch_description():

    pkg_name = 'robodog'
    file_subpath = 'urdf/robodog_robot.urdf.xacro'
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'robodog_rviz_config.rviz')
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    check_file_exists(xacro_file, 'xacro file')
    check_file_exists(rviz_config_file, 'RViz config file')

    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    ros2_control_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'robodog_controller.yaml')
    check_file_exists(ros2_control_params, 'ROS2 control parameters')

    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    check_file_exists(gazebo_launch_file, 'Gazebo launch file')

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )#!!!!

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw}, {'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'robodog'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_raw, ros2_control_params],  
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller'],  
        output='screen'
    )

    robodog_position_publisher_node = Node(
        package='robodog',  
        executable='robodog_position_publisher_node.py',
        name='robodog_position_publisher_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        controller_manager_node,
        spawner_node,    
        robodog_position_publisher_node,  
        rviz_node, 
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('midterm_robot')
    urdf_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'description', 'road.sdf') # Uses your custom road

    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'custom_robot'], output='screen'
    )

    rviz = Node(package='rviz2', executable='rviz2', output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([gazebo, robot_state_publisher, spawn_entity, rviz, bridge])

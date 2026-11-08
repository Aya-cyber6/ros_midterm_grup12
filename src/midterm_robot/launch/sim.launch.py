from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('midterm_robot')
    urdf_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'description', 'road.sdf')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Restored: Required for wheels to appear in RViz
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'custom_robot',
            '-z', '0.2'  # FIX: Drops the robot from the air so it doesn't clip into the floor
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/bumper_states@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            '/sonar/range@sensor_msgs/msg/Range@gz.msgs.LaserScan', # Restored Ramazan's Sonar
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
        rviz,
        bridge
    ])
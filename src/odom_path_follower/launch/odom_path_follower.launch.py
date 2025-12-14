from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('odom_path_follower')
    config_path = PathJoinSubstitution([package_share, 'config', 'odom_path_follower.yaml'])

    ekf_odom_node = Node(
        package='odom_path_follower',
        executable='ekf_odom',
        name='ekf_odom',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        ekf_odom_node
    ])
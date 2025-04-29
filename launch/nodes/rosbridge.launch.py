from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Get the package share directory
    pkg_share = get_package_share_directory('luggage_av')  # Replace with your package name
    params_file_full_path = os.path.join(pkg_share, 'parameters', 'rosbridge_params.yaml')

    namespace = LaunchConfiguration('namespace')

    # Define launch arguments
    args = GroupAction([
        DeclareLaunchArgument('namespace', default_value='luggage_av', description='Top-level namespace'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
    ])

    # Define nodes
    rosbrigde_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            params_file_full_path,
            {
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
            },
        ],
        namespace=namespace,
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        output='screen',
        parameters=[
            params_file_full_path,
        ],
        namespace=namespace,
    )

    return LaunchDescription([
        args,
        rosbrigde_node,
        rosapi_node
    ])

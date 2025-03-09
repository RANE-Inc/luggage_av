from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('luggage_av')  # Replace with your package name
    params_file_full_path = os.path.join(pkg_share, 'parameters', 'rosbridge_params.yaml')

    namespace = LaunchConfiguration('namespace')
    param_substitutions = {
        'rosbridge_websocket.ros__parameters.port': LaunchConfiguration('port'),
        'rosbridge_websocket.ros__parameters.address': LaunchConfiguration('address'),
        'rosbridge_websocket.ros__parameters.certfile': LaunchConfiguration('certfile'),
        'rosbridge_websocket.ros__parameters.keyfile': LaunchConfiguration('keyfile'),
        # Glob patterns for filtering and other Launch Configs can be exposed
    }

    # Define launch arguments
    args = [
        DeclareLaunchArgument('namespace', default_value='luggage_av', description='Top-level namespace'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
    ]

    # Define nodes
    rosbrigde_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        namespace=namespace,
        output='screen',
        parameters=[params_file_full_path, param_substitutions],
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file_full_path, param_substitutions],
    )

    return LaunchDescription(args + [rosbrigde_node, rosapi_node])
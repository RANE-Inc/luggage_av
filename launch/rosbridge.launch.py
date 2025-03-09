from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('luggage_av')  # Replace with your package name
    params_file_full_path = os.path.join(pkg_share, 'parameters', 'rosbridge_params.yaml')

    namespace = LaunchConfiguration('namespace')
    ssl = LaunchConfiguration('ssl')
    param_substitutions = {
        'rosbridge_websocket.ros__parameters.port': LaunchConfiguration('port'),
        'rosbridge_websocket.ros__parameters.address': LaunchConfiguration('address'),
        'rosbridge_websocket.ros__parameters.ssl': ssl,
        'rosbridge_websocket.ros__parameters.certfile': LaunchConfiguration('certfile'),
        'rosbridge_websocket.ros__parameters.keyfile': LaunchConfiguration('keyfile'),
        # Glob patterns for filtering and other Launch Configs can be exposed
    }

    # Process parameters with substitutions and namespace
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    # Define launch arguments
    args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('params_file', default_value=params_file_full_path, description='Full path to the ROS2 parameters file for rosbridge'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
    ]

    # Define nodes
    ssl_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        condition=IfCondition(ssl)
    )

    non_ssl_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        condition=UnlessCondition(ssl)
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    return LaunchDescription(args + [ssl_node, non_ssl_node, rosapi_node])
#       This is a pythong recreation of rosbridge_websocket_launch.xml
# I tried making a light version and it was not working, but "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" was working
# So I decided to recreate the launch file in python

# Tested by running
#           wscat -c ws://localhost:9090
# Enter value:
#           {  "op": "call_service",  "service": "/rosapi/topics",  "args": {}}
# Response:
#           < {"op": "service_response", "service": "/rosapi/topics", "values": {"topics": ["/client_count", "/connected_clients", "/parameter_events", "/rosout"], "types": ["std_msgs/msg/Int32", "rosbridge_msgs/msg/ConnectedClients", "rcl_interfaces/msg/ParameterEvent", "rcl_interfaces/msg/Log"]}, "result": true}


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Define all launch arguments
    args = [
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),
        DeclareLaunchArgument('fragment_timeout', default_value='600'),
        DeclareLaunchArgument('delay_between_messages', default_value='0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),
        DeclareLaunchArgument('use_compression', default_value='false'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='false'),
        DeclareLaunchArgument('send_action_goals_in_new_thread', default_value='false'),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('bson_only_mode', default_value='false')
    ]

    # Common parameters for both SSL and non-SSL
    common_params = {
        'port': LaunchConfiguration('port'),
        'address': LaunchConfiguration('address'),
        'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
        'fragment_timeout': LaunchConfiguration('fragment_timeout'),
        'delay_between_messages': LaunchConfiguration('delay_between_messages'),
        'max_message_size': LaunchConfiguration('max_message_size'),
        'unregister_timeout': LaunchConfiguration('unregister_timeout'),
        'use_compression': LaunchConfiguration('use_compression'),
        'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
        'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
        'topics_glob': LaunchConfiguration('topics_glob'),
        'services_glob': LaunchConfiguration('services_glob'),
        'params_glob': LaunchConfiguration('params_glob')
    }

    # SSL-specific node
    ssl_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            **common_params,
            'ssl': LaunchConfiguration('ssl'),
            'certfile': LaunchConfiguration('certfile'),
            'keyfile': LaunchConfiguration('keyfile')
        }],
        condition=IfCondition(LaunchConfiguration('ssl'))
    )

    # Non-SSL node (includes bson_only_mode)
    non_ssl_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            **common_params,
            'ssl': LaunchConfiguration('ssl'),
            'bson_only_mode': LaunchConfiguration('bson_only_mode')
        }],
        condition=UnlessCondition(LaunchConfiguration('ssl'))
    )

    # Rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        parameters=[{
            'topics_glob': LaunchConfiguration('topics_glob'),
            'services_glob': LaunchConfiguration('services_glob'),
            'params_glob': LaunchConfiguration('params_glob')
        }]
    )

    return LaunchDescription(args + [ssl_node, non_ssl_node, rosapi_node])
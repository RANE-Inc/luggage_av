import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('luggage_av')

    namespace = LaunchConfiguration("namespace")

    # Define the path to the behavior tree XML file
    bt_xml_path = os.path.join(package_dir, 'behavior_trees', 'main_bt.xml')

    # Node definitions
    behavior_tree_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {'bt_xml_filename': bt_xml_path},
            {'plugins': ['luggage_av::RegisterNodes']}
        ],
        arguments=['--ros-args', '--params-file', os.path.join(package_dir, 'parameters', 'bt_navigator_params.yaml')],
        namespace=["/", namespace]
    )

    lifecycle_nodes = ['bt_navigator']
    autostart = True

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),
        behavior_tree_node,
        lifecycle_manager
    ])
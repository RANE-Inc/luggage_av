import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    namespace = LaunchConfiguration("namespace")

    game_controller_node = Node(
        name="game_controller_node",
        package="joy",
        executable="game_controller_node",
        namespace=["/", namespace],
    )
    teleop_twist_joy_node = Node(
        name="teleop_node",
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[
            os.path.join(pkg_share, "parameters", "teleop_twist_joy.yaml"),
            {
                "frame": namespace, # TODO: Remove prefix to allow for stricter namespace passing
            },
        ],
        namespace=["/", namespace],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        game_controller_node,
        teleop_twist_joy_node,
    ])

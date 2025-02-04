import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, EqualsSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from lifecycle_msgs.msg import Transition



def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    sim_mode = LaunchConfiguration("sim_mode")
    map_filename = LaunchConfiguration("map_filename")
    namespace = LaunchConfiguration("namespace")

    # TODO: map files from world argument
    slam_toolbox = LifecycleNode(
        parameters=[
          os.path.join(pkg_share, "parameters", "slam_toolbox.yaml"),
          {
            "use_sim_time": sim_mode,
            # "map_file_name": map_filename,
            "odom_frame": IfElseSubstitution(EqualsSubstitution(namespace, ""), "odom", [namespace, "/odom"]), # TODO: Remove prefix to allow for stricter namespace passing
            "map_frame": IfElseSubstitution(EqualsSubstitution(namespace, ""), "map", [namespace, "/map"]), # TODO: Remove prefix to allow for stricter namespace passing
            "base_frame": IfElseSubstitution(EqualsSubstitution(namespace, ""), "base_footprint", [namespace, "/base_footprint"]), # TODO: Remove prefix to allow for stricter namespace passing
          },
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",       # TODO: check if online_async_launch
        name="slam_toolbox",
        output="screen",
        namespace=["/", namespace],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan", "scan"),
            ("/map", "map"),
            ("/map_updates", "map_updates"),
            ("/map_metadata", "map_metadata"),
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "map_filename",
            default_value="",
            description="Full path to the parameters file to use for Slam Toolbox"
        ),
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        slam_toolbox,

        EmitEvent(
            event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE
            ),
        ),
        RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
    )
    ])

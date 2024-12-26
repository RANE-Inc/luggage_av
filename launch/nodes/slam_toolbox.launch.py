import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from lifecycle_msgs.msg import Transition



def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    sim_mode = LaunchConfiguration("sim_mode")
    map_filename = LaunchConfiguration('map_filename')
    namespaced_tf = LaunchConfiguration("namespaced_tf")

    slam_toolbox = LifecycleNode(
        parameters=[
          os.path.join(pkg_share, 'parameters', 'slam_toolbox.yaml'),
          {
            'use_sim_time': sim_mode,
            # "map_file_name": map_filename,
          },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',       # TODO check if online_async_launch
        name='slam_toolbox',
        output='screen',
        namespace='luggage_av',
        remappings=[
            ('/map','/luggage_av/map'),
            ('/map_updates','/luggage_av/map_updates'),
            ('/map_metadata', '/luggage_av/map_metadata'),
        ],
        condition=UnlessCondition(namespaced_tf)
    )

    slam_toolbox_ns_tf = LifecycleNode(
        parameters=[
          os.path.join(pkg_share, 'parameters', 'slam_toolbox.yaml'),
          {
            'use_sim_time': sim_mode,
            # "map_file_name": map_filename,
          },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='luggage_av',
        remappings=[
            ('/tf', 'tf'), 
            ('/tf_static', 'tf_static'),
            ('/map','/luggage_av/map'),
            ('/map_updates','/luggage_av/map_updates'),
            ('/map_metadata', '/luggage_av/map_metadata'),
        ],
        condition=IfCondition(namespaced_tf)
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_filename',
            default_value="",
            description='Full path to the parameters file to use for Slam Toolbox'
        ),
        DeclareLaunchArgument(
            "sim_mode", 
            default_value="false"
        ),
        DeclareLaunchArgument(
            'namespaced_tf',
            default_value="false",
            description="Whether to use namespaced tf topic aka /ns/tf and /ns/tf_static"
        ), 

        slam_toolbox,
        slam_toolbox_ns_tf,

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

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    sim_mode = LaunchConfiguration('sim_mode')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        # 'smoother_server',
        # 'planner_server',
        # 'behavior_server',
        # 'velocity_smoother',
        # 'collision_monitor',
        # 'bt_navigator',
        # 'waypoint_follower',
        # 'docking_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')] # FIXME: can't use namespaced tf, as it's not supported by controller_manager spawner
    remappings = [] 

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'autostart': "true",
        'use_sim_time': sim_mode,
        }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(pkg_share, "parameters", "nav2_params.yaml"),
            root_key="luggage_av",
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', sim_mode),
            PushROSNamespace(namespace="luggage_av"),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            # Node(
            #     package='nav2_smoother',
            #     executable='smoother_server',
            #     name='smoother_server',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            # Node(
            #     package='nav2_planner',
            #     executable='planner_server',
            #     name='planner_server',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            # Node(
            #     package='nav2_behaviors',
            #     executable='behavior_server',
            #     name='behavior_server',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            # ),
            # Node(
            #     package='nav2_bt_navigator',
            #     executable='bt_navigator',
            #     name='bt_navigator',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            # Node(
            #     package='nav2_waypoint_follower',
            #     executable='waypoint_follower',
            #     name='waypoint_follower',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            # Node(
            #     package='nav2_velocity_smoother',
            #     executable='velocity_smoother',
            #     name='velocity_smoother',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings
            #     + [('cmd_vel', 'cmd_vel_nav')],
            # ),
            # Node(
            #     package='nav2_collision_monitor',
            #     executable='collision_monitor',
            #     name='collision_monitor',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            # Node(
            #     package='opennav_docking',
            #     executable='opennav_docking',
            #     name='docking_server',
            #     output='screen',
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='log level'
        ),

        load_nodes
    ])

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld

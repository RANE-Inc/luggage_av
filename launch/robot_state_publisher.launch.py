import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    sim_mode = LaunchConfiguration("sim_mode")
    namespaced_tf = LaunchConfiguration("namespaced_tf")

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": PythonExpression(
                    [
                        "xacro.process_file('",
                        os.path.join(pkg_share, "urdf", "robot.urdf.xacro"),
                        "',mappings={'sim_mode':'",
                        EqualsSubstitution(sim_mode, 'true'), # Sanitize input
                        "'}).toxml()"
                    ],
                    ['xacro']
                )
            },
            os.path.join(pkg_share, "parameters", "robot_state_publisher.yaml"),    
        ],
        namespace="luggage_av",
        condition=UnlessCondition(namespaced_tf),
    )

    robot_state_publisher_node_ns_tf = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": PythonExpression(
                    [
                        "xacro.process_file('",
                        os.path.join(pkg_share, "urdf", "robot.urdf.xacro"),
                        "',mappings={'sim_mode':'",
                        EqualsSubstitution(sim_mode, 'true'), # Sanitize input
                        "'}).toxml()"
                    ],
                    ['xacro']
                )
            },
            os.path.join(pkg_share, "parameters", "robot_state_publisher.yaml"),    
        ],
        remappings=[('/tf', 'tf'), ('/tf_static','tf_static')],
        namespace="luggage_av",
        condition=IfCondition(namespaced_tf)
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_mode", 
            default_value="false",
            description="Set to true when running in a simulator"
        ),
        DeclareLaunchArgument(
            'namespaced_tf',
            default_value="false",
            description="Whether to use namespaced tf topic aka /ns/tf and /ns/tf_static"
        ), 

        robot_state_publisher_node
    ])

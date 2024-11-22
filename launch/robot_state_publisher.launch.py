import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, EqualsSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    sim_mode = LaunchConfiguration("sim_mode")

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
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_mode", 
            default_value="false"
        ),

        robot_state_publisher_node
    ])

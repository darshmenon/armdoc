"""ros2 launch armdoc armdoc.launch.py [ns:=/ur3] [moveit:=true] [diagnostics:=true] [use_sim_time:=true]

Runs the live doctor (a report every `period` seconds). Thresholds come from config/armdoc.yaml, or your own file
via params_file:=...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("armdoc"), "config", "armdoc.yaml")
    args = [
        DeclareLaunchArgument("ns", default_value="", description="namespace of the arm, e.g. /ur3"),
        DeclareLaunchArgument("moveit", default_value="false", description="also check move_group and the planning scene"),
        DeclareLaunchArgument("diagnostics", default_value="false", description="also publish findings to /diagnostics"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="true when the stamps are simulation time"),
        DeclareLaunchArgument("params_file", default_value=default_params, description="thresholds YAML"),
    ]
    node = Node(
        package="armdoc", executable="armdoc", name="armdoc", output="screen", emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), {
            "ns": LaunchConfiguration("ns"),
            "moveit": ParameterValue(LaunchConfiguration("moveit"), value_type=bool),
            "diagnostics": ParameterValue(LaunchConfiguration("diagnostics"), value_type=bool),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])

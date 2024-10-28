import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    joy_teleop = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("teleop_twist_joy"),
            "launch",
            "teleop-launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    joy_node = Node(
        package="bumperbot_py_examples",
        executable="joystick",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
        ]
    )

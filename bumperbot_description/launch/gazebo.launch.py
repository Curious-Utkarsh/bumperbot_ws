import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    default_world = os.path.join(
        get_package_share_directory("bumperbot_description"),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumperbot_description).parent.resolve())
            ]
        )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"
                    "-x", "0.0",  # Set your desired x position
                    "-y", "0.0",  # Set your desired y position
                    "-z", "0.0",  # Set your desired z position
                    "-R", "0.0",  # Set your desired roll
                    "-P", "0.0",  # Set your desired pitch
                    "-Y", "-1.57"  # Set your desired yaw
                   ],
    )

    gz_spawn_sdf_model = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", os.path.join(bumperbot_description, "models", "maze_qr_codes", "model.sdf"),
            "-name", "my_qr_maze_model",  # Name for your model in Gazebo
            "-x", "0", "-y", "0", "-z", "0"  # Adjust position as needed
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    bridge_params = os.path.join(get_package_share_directory("bumperbot_description"),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    gz_ros2_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        world_arg,
        gazebo,
        gz_spawn_entity,
        gz_spawn_sdf_model,
        gz_ros2_bridge,
        ros_gz_bridge,
        gz_ros2_image_bridge,
    ])
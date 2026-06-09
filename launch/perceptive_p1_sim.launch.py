from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    perceptive_share = FindPackageShare("perceptive_legged_control")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("legged_gazebo"), "launch", "p1_sim.launch.py"])
        ),
        launch_arguments={
            "task_file": PathJoinSubstitution([perceptive_share, "config", "p1", "task.info"]),
            "reference_file": PathJoinSubstitution([perceptive_share, "config", "p1", "reference.info"]),
            "legged_controller_type": "perceptive_legged_control/PerceptiveLeggedController",
            "gui": gui,
        }.items(),
    )

    perceptive_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([perceptive_share, "launch", "perceptive_stack.launch.py"])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        simulation,
        perceptive_stack,
    ])

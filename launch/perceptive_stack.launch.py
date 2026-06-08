from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameter_file = LaunchConfiguration("parameter_file")
    node_parameter_file = LaunchConfiguration("node_parameter_file")
    world_file = LaunchConfiguration("world_file")
    target_frame_id = LaunchConfiguration("target_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    use_robot_submap = LaunchConfiguration("use_robot_submap")
    map_padding = LaunchConfiguration("map_padding")
    use_sim_time = LaunchConfiguration("use_sim_time")
    target_config = LaunchConfiguration("target_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "parameter_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("convex_plane_decomposition_ros"),
                "config",
                "parameters.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "node_parameter_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("convex_plane_decomposition_ros"),
                "config",
                "world_box_terrain.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("legged_gazebo"),
                "worlds",
                "empty_world.world",
            ]),
        ),
        DeclareLaunchArgument("target_frame_id", default_value="odom"),
        DeclareLaunchArgument("base_frame_id", default_value="base"),
        DeclareLaunchArgument("use_robot_submap", default_value="false"),
        DeclareLaunchArgument("map_padding", default_value="3.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "target_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("perceptive_legged_control"),
                "config",
                "perceptive_target.yaml",
            ]),
        ),
        Node(
            package="convex_plane_decomposition_ros",
            executable="convex_plane_decomposition_ros_world_box_terrain",
            name="convex_plane_decomposition_ros",
            output="screen",
            parameters=[
                parameter_file,
                node_parameter_file,
                {
                    "world_file": world_file,
                    "target_frame_id": target_frame_id,
                    "base_frame_id": base_frame_id,
                    "use_robot_submap": use_robot_submap,
                    "map_padding": map_padding,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=[
                ("planar_terrain", "/convex_plane_decomposition_ros/planar_terrain"),
                ("filtered_map", "/convex_plane_decomposition_ros/filtered_map"),
                ("boundaries", "/convex_plane_decomposition_ros/boundaries"),
                ("insets", "/convex_plane_decomposition_ros/insets"),
            ],
        ),
        Node(
            package="perceptive_legged_control",
            executable="perceptive_target_trajectories_publisher",
            name="perceptive_target_trajectories_publisher",
            output="screen",
            parameters=[target_config],
        ),
    ])

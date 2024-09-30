import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

import launch


def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        "main_param_dir",
        default=os.path.join(
            get_package_share_directory("local_navigation"),
            "config",
            "lidarslam_summit.yaml",
        ),
    )

    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        "rviz_param_dir",
        default=os.path.join(
            get_package_share_directory("local_navigation"), "config", "local_nav.rviz"
        ),
    )

    mapping = launch_ros.actions.Node(
        package="scanmatcher",
        executable="scanmatcher_node",
        parameters=[main_param_dir],
        remappings=[("/input_cloud", "/front_laser/points")],
        output="screen",
    )

    graphbasedslam = launch_ros.actions.Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        parameters=[main_param_dir],
        output="screen",
    )

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        parameters=[main_param_dir],
        arguments=["-d", rviz_param_dir],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "main_param_dir",
                default_value=main_param_dir,
                description="Full path to main parameter file to load",
            ),
            mapping,
            graphbasedslam,
            rviz,
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile

def launch_setup(context, *args, **kwargs):
    """Set up and configure radar nodes."""
    
    def create_parameter_dict(*args):
        return {x: LaunchConfiguration(x) for x in args}

    # Radar Parameters
    radar_params = create_parameter_dict(
        "radar_raw_name",
        "radar_filtered_name",
        "frame_id",
    )

    # Define Radar Nodes
    radar_fitler_node_param = ParameterFile(
        param_file=LaunchConfiguration("radar_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    radar_msg_converter_node_param = ParameterFile(
        param_file=LaunchConfiguration("radar_msg_converter_node_param_path").perform(context),
        allow_substs=True,
    )
    nodes = [
        ComposableNode(
            package="radar_tracks_noise_filter",
            plugin="radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode",
            name="radar_tracks_noise_filter",
            remappings=[
                ("~/input/tracks", "/sensing/radar_1/objects_raw"),
                ("~/output/filtered_tracks", "/sensing/radar_1/filtered_objects"),
            ],
            parameters=[radar_fitler_node_param]
        ),
        ComposableNode(
            package="radar_tracks_msgs_converter",
            plugin="radar_tracks_msgs_converter::RadarTracksMsgsConverterNode",
            name="radar_tracks_msgs_converter",
            remappings=[
                ("~/input/radar_objects", "/sensing/radar_1/filtered_objects"),
                ("~/input/odometry", "/localization/kinematic_state"),
                ("~/output/radar_detected_objects", "/sensing/radar_1/detected_objects_conv"),
                ("~/output/radar_tracked_objects", "/sensing/radar_1/tracked_objects"),
            ],
            parameters=[radar_msg_converter_node_param]
        )
    ]

    # Radar Container
    container = ComposableNodeContainer(
        name="radar_container",
        namespace="sensing",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]


def generate_launch_description():
    """Generate the radar processing pipeline launch description."""
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value, description=description))
    
    
    autoware_radar_tracks_noise_filter_dir = get_package_share_directory("radar_tracks_noise_filter")
    autoware_radar_msg_converter_dir = get_package_share_directory("radar_tracks_msgs_converter")


    add_launch_arg(
        "radar_filter_node_param_path",
        os.path.join(
            autoware_radar_tracks_noise_filter_dir,
            "config",
            "radar_tracks_noise_filter.param.yaml",
        ),
        description="path to parameter file of distortion correction node",
    )
    add_launch_arg(
        "radar_msg_converter_node_param_path",
        os.path.join(
            autoware_radar_msg_converter_dir,
            "config",
            "radar_tracks_msgs_converter.param.yaml",
        ),
        description="path to parameter file of distortion correction node",
    )

    # Radar Topics & Configurations
    add_launch_arg("radar_raw_name", "/sensing/radar_1/objects_raw", "Raw radar object topic")
    add_launch_arg("radar_filtered_name", "/sensing/radar_1/filtered_objects", "Filtered radar objects")
    add_launch_arg("frame_id", "radar_1", "Radar frame ID")
    add_launch_arg("container_executable", "component_container", "Executable for the container")

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])

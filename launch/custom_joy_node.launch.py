from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    node_name = LaunchConfiguration("node_name")
    joy_topic = LaunchConfiguration("joy_topic")
    device_path = LaunchConfiguration("device_path")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    scan_period_ms = LaunchConfiguration("scan_period_ms")
    default_axes_count = LaunchConfiguration("default_axes_count")
    default_buttons_count = LaunchConfiguration("default_buttons_count")
    deadzone = LaunchConfiguration("deadzone")
    prefer_evdev = LaunchConfiguration("prefer_evdev")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("custom_joy_node"), "config", "custom_joy_node.param.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config_file),
            DeclareLaunchArgument("node_name", default_value="custom_joy_node"),
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("device_path", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("scan_period_ms", default_value="1000"),
            DeclareLaunchArgument("default_axes_count", default_value="8"),
            DeclareLaunchArgument("default_buttons_count", default_value="16"),
            DeclareLaunchArgument("deadzone", default_value="0.05"),
            DeclareLaunchArgument("prefer_evdev", default_value="false"),
            Node(
                package="custom_joy_node",
                executable="custom_joy_node",
                name=node_name,
                output="screen",
                parameters=[
                    config_file,
                    {
                        "device_path": device_path,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "scan_period_ms": ParameterValue(scan_period_ms, value_type=int),
                        "default_axes_count": ParameterValue(default_axes_count, value_type=int),
                        "default_buttons_count": ParameterValue(default_buttons_count, value_type=int),
                        "deadzone": ParameterValue(deadzone, value_type=float),
                        "prefer_evdev": ParameterValue(prefer_evdev, value_type=bool),
                    },
                ],
                remappings=[
                    ("/joy", joy_topic),
                ],
            ),
        ]
    )

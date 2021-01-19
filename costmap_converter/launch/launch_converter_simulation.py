#  (C) Copyright
#  Logivations GmbH, Munich 2010-2020
import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = LaunchConfiguration('params_file')
    remappings = [((namespace, '/tf'), '/tf'),
                  ((namespace, '/tf_static'), '/tf_static'),
                  ('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)
    return launch.LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'rviz_prefix', default_value='',
            description='Top-level rviz prefix'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'costmap_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        launch_ros.actions.Node(
            package='costmap_converter', node_executable='standalone_converter', output='screen',
            parameters=[{"rolling_window": False,
                         "map_topic": ("/", LaunchConfiguration("rviz_prefix") , "server_costmap_node/map"),
                         "static_layer.subscribe_to_updates": True,
                         "static_layer.map_subscribe_transient_local": True
                         }],
        ),
    ])

"""Start the controller and all simulation/display nodes.

ros2 launch ugv_patrol_planner patrol.launch.py [scenario:=<file>] [gui:=true|false] [rviz:=true|false] [snapshot:=<png>]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'ugv_patrol_planner'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('scenario', default_value=os.path.join(share, 'scenarios', 'default.yaml')),
        DeclareLaunchArgument('gui', default_value='true', description='open the OpenCV window'),
        DeclareLaunchArgument('rviz', default_value='false', description='also start RViz2'),
        DeclareLaunchArgument('snapshot', default_value='', description='save the 2D view to this PNG every second'),
        DeclareLaunchArgument('drive_rate', default_value='5.0', description='UGV moves per second (tests use more)'),
    ]
    common = {'config': LaunchConfiguration('config'), 'scenario': LaunchConfiguration('scenario')}

    def node(executable, extra=None):
        return Node(package=PKG, executable=executable, name=executable, output='screen',
                    parameters=[{**common, **(extra or {})}])

    return LaunchDescription(args + [
        node('controller'),
        node('drive_sim', {'rate_hz': ParameterValue(LaunchConfiguration('drive_rate'), value_type=float)}),
        node('battery_sim'),
        node('perception_sim'),
        node('visualizer', {
            'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
            'snapshot': ParameterValue(LaunchConfiguration('snapshot'), value_type=str),
        }),
        # The markers use the 'map' frame; publish it so RViz2 has a valid fixed frame.
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_frame', output='log',
             arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'patrol.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

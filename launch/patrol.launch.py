"""Start the controller and all simulation/display nodes.

ros2 launch ugv_patrol_planner patrol.launch.py [scenario:=<file>] [gui:=true|false] [rviz:=true|false] [snapshot:=<png>]
                                                 [controller:=py|cpp]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        DeclareLaunchArgument('snapshot', default_value='',
                              description='save the 2D view to this PNG (use %d in the name for numbered frames)'),
        DeclareLaunchArgument('snapshot_hz', default_value='1.0', description='snapshots per second'),
        DeclareLaunchArgument('drive_rate', default_value='5.0', description='UGV moves per second (tests use more)'),
        DeclareLaunchArgument('controller', default_value='py', description='py (rclpy) or cpp (rclcpp controller)'),
    ]
    common = {'config': LaunchConfiguration('config'), 'scenario': LaunchConfiguration('scenario')}

    def node(executable, extra=None, package=PKG, condition=None):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[{**common, **(extra or {})}], condition=condition)

    def controller_is(lang):
        return IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), f"' == '{lang}'"]))

    return LaunchDescription(args + [
        # Same node name, topics and QoS in both languages; pick one with controller:=py|cpp.
        node('controller', condition=controller_is('py')),
        node('controller', package=PKG + '_cpp', condition=controller_is('cpp')),
        node('drive_sim', {'rate_hz': ParameterValue(LaunchConfiguration('drive_rate'), value_type=float)}),
        node('battery_sim'),
        node('perception_sim'),
        node('visualizer', {
            'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
            'snapshot': ParameterValue(LaunchConfiguration('snapshot'), value_type=str),
            'snapshot_hz': ParameterValue(LaunchConfiguration('snapshot_hz'), value_type=float),
        }),
        # The markers use the 'map' frame; publish it so RViz2 has a valid fixed frame.
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_frame', output='log',
             arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        OpaqueFunction(function=rviz_node, kwargs={'share': share}, condition=IfCondition(LaunchConfiguration('rviz'))),
    ])


def rviz_node(context, share):
    """RViz2 with the camera and grid fitted to this run's map (the template is sized for one map only)."""
    import tempfile

    import numpy as np

    from ugv_patrol_planner.common import load_config
    from ugv_patrol_planner.rviz_view import write_fitted

    def resolve(path):
        return path if os.path.isabs(path) else os.path.join(share, path)

    template = os.path.join(share, 'rviz', 'patrol.rviz')
    rviz_file = template
    try:
        cfg = load_config(resolve(LaunchConfiguration('config').perform(context)))
        dem = np.load(resolve(cfg.dem_file))
        rviz_file = os.path.join(tempfile.gettempdir(), 'ugv_patrol_planner.rviz')
        write_fitted(template, rviz_file, cfg.map_size, cfg.cell_size_m,
                     ground_z=float(dem.min()) - 1.0, focus_z=float(dem.mean()))  # grid just under the terrain
    except Exception as e:  # keep RViz usable with the fixed template
        print(f'[patrol.launch] could not fit the RViz view to the map, using {template}: {e}')
    return [Node(package='rviz2', executable='rviz2', name='rviz2', output='log', arguments=['-d', rviz_file])]

from glob import glob

from setuptools import find_packages, setup

package_name = 'ugv_patrol_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # share/ in the repo holds what is installed into the ROS 2 share directory as it is.
    data_files=[
        ('share/ament_index/resource_index/packages', ['share/resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('share/config/*.yaml')),
        ('share/' + package_name + '/launch', glob('share/launch/*.launch.py')),
        ('share/' + package_name + '/scenarios', glob('share/scenarios/*.yaml')),
        ('share/' + package_name + '/data', glob('share/data/*.npy')),
        ('share/' + package_name + '/data/campus_osm', glob('share/data/campus_osm/*.npy')),
        ('share/' + package_name + '/data/campus_pred', glob('share/data/campus_pred/*.npy')),
        ('share/' + package_name + '/rviz', glob('share/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ugv_patrol_planner maintainer',
    maintainer_email='maintainer@example.com',
    description='2.5D terrain UGV patrol path planning controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = ugv_patrol_planner.nodes.controller_node:main',
            'drive_sim = ugv_patrol_planner.nodes.drive_sim:main',
            'battery_sim = ugv_patrol_planner.nodes.battery_sim:main',
            'perception_sim = ugv_patrol_planner.nodes.perception_sim:main',
            'visualizer = ugv_patrol_planner.nodes.visualizer:main',
        ],
    },
)

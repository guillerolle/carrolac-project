import os
import datetime

from ament_index_python.packages import (get_package_share_path, get_package_share_directory, get_package_prefix,
                                         PackageNotFoundError)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, ExecuteProcess, \
    RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros2pkg.api import PackageNotFound


# from ignition.common import set_verbosity


def generate_launch_description():
    returnList = []
    default_package_path = get_package_share_path('computer_vision')

    # ROSBAG
    returnList.append(DeclareLaunchArgument(name='bag_on', default_value="False",
                                            description='Indica si se cargará un bag o no',
                                            choices=["False", "True"]))
    returnList.append(DeclareLaunchArgument(name='bag_file',
                                            default_value=str(os.path.join('bags',
                                                                           '2024_02_23-18_43_03')),
                                            description='Path bag file'))

    returnList.append(ExecuteProcess(
        condition=ParameterValue(LaunchConfiguration("bag_on"), value_type=bool),
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file')],
        output='screen',
    ))

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(default_package_path, 'rviz', 'computer_vision.rviz')
        ]
    )
    returnList.append(rviz)

    # COMPUTER VISION
    returnList.append(ExecuteProcess(
        cmd=['src/computer_vision/computer_vision/aruco_detector.py', '--ros-args', '-p', 'use_sim_time:=True'],
        output='screen'
    ))

    return LaunchDescription(returnList)

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


def generate_launch_description():
    returnList = []
    default_package_path = get_package_share_path('carrolac_gazebo')
    default_model_path = default_package_path / 'urdf/carrolac_2difs.urdf.xacro'

    description_package_path = get_package_share_path('carrolac_description')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    returnList.append(model_arg)

    sim_on_arg = DeclareLaunchArgument(name='sim_on', default_value="True",
                                       description='Indica si la simulación empieza ejecutandose o pausada',
                                       choices=["False", "True"])
    returnList.append(sim_on_arg)

    trajectory_control_arg = DeclareLaunchArgument(name='trajectory_control', default_value="False",
                                                   description='Determina el uso de control de juntas '
                                                               '"JointTrajectoryControl" o controladores '
                                                               'independientes para cada una',
                                                   choices=["False", "True"])
    returnList.append(trajectory_control_arg)

    qini_arg = DeclareLaunchArgument(name='qini', default_value="[0,0,0,0,0]",
                                     description='Condiciones Iniciales: [x, y, theta, joint_arm_1, joint_arm_2]',
                                     )
    returnList.append(qini_arg)

    robot_description = ParameterValue(Command(
        ['xacro "', LaunchConfiguration('model'), '"', ]), value_type=str)

    # Robot State Publisher
    params = {'use_sim_time': True, 'robot_description': robot_description}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[]
    )
    returnList.append(robot_state_publisher)

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(description_package_path, 'rviz', 'urdf.rviz')
        ]
    )
    returnList.append(rviz)

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [PythonExpression(['"-r " if ', LaunchConfiguration('sim_on'), ' else " "']),
                                      " --record ", "--verbose 4 ",
                                      os.path.join(default_package_path, 'worlds', 'factory.sdf')
                                      ]
                          }.items()
    )
    returnList.append(gazebo)

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                     '-name', 'carrolac',
                     '-x', PythonExpression([LaunchConfiguration('qini'), '[0]']),
                     '-z', '0.5',
                     '-y', PythonExpression([LaunchConfiguration('qini'), '[1]']),
                     '-topic', '/robot_description',
                 ],
                 output='screen')
    returnList.append(spawn)

    # Gz - ROS Bridge
    bridgeDefaults = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (IGN -> ROS2)
            '/world/empty/model/carrolac/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/carrolac/joint_state', 'joint_states'),
        ],
        output='screen'
    )
    returnList.append(bridgeDefaults)

    # Puente World Pose GZ -> ROS, biblioteca ROSLAC
    # LogInfo(
    #     condition=IfCondition(Command([
    #         FindExecutable(name='ros2'),'pkg','prefix','roslac_gz_bridge'])),
    #     msg="Existe Paquete roslac_gz_bride",
    # )
    try:
        get_package_prefix('roslac_gz_bridge')
        bridgeWorldPoseLAC = Node(
            # condition=IfCondition(Command([
            #     FindExecutable(name='ros2'), ' pkg', ' prefix', ' roslac_gz_bridge'
            # ])),
            package='roslac_gz_bridge',
            executable='world_pose',
        )
        returnList.append(bridgeWorldPoseLAC)
    except PackageNotFoundError:
        # bridgeWorldPoseLAC = None
        pass

    bridgeJointCommands = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Joint Commands (ROS2 -> GZ)
            # '/model/scorbot/joint/joint_one/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            # ('/model/scorbot/joint/joint_one/cmd_pos', '/commands/joint_one'),
        ],
        output='screen'
    )
    returnList.append(bridgeJointCommands)

    # CAMERA BRIDGE
    bridgeCamera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        # remappings=[
        #     (
        #         # '/world/empty/model/carrolac/joint_state', 'joint_states',
        #     ),
        # ],
        output='screen'
    )
    returnList.append(bridgeCamera)

    # LIDAR BRIDGE
    returnList.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        # remappings=[
        #     (
        #         # '/world/empty/model/carrolac/joint_state', 'joint_states',
        #     ),
        # ],
        output='screen'
    ))

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    returnList.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[load_joint_state_broadcaster],
        )
    ), )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    returnList.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],
        )
    ), )

    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster'],
        output='screen'
    )

    returnList.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_trajectory_controller,
            on_exit=[load_imu_sensor_broadcaster],
        )
    ), )

    # TELEOP
    returnList.append(ExecuteProcess(
        cmd=['ros2', 'run', 'carrolac_kboard', 'teleop'],
        output='screen'
    ))

    # ROSBAG
    bags_directory = 'bags.NOSYNC'
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    returnList.append(ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '--output', bags_directory + '/' + timestamp, '--use-sim-time'],
        output='screen',
    ))

    return LaunchDescription(returnList)

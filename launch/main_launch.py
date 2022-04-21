import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    # Launch args
    # prefix = LaunchConfiguration('prefix')

    # config_husky_velocity_controller = PathJoinSubstitution(
    #     [FindPackageShare("husky_control"), "config", "control.yaml"]
    # )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dual_arm_husky_description"), "urdf", "dual_arm_husky.urdf.xacro"]
            ),
            " ",
            "name:=husky"
        ]
    )
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
    #         ),
    #         " ",
    #         "name:=husky",
    #         " ",
    #         "prefix:=''",
    #         " ",
    #         "is_sim:=true",
    #         " ",
    #         "gazebo_controllers:=",
    #         config_husky_velocity_controller,
    #     ]
    # )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r empty.sdf'
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_husky',
        arguments=['-topic', 'robot_description',
          '-z', '0.15'],
        output='screen',
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    spawn_joint_state_broadcaster_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_joint_state_broadcaster]
        )
    )

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    spawn_husky_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(ign_gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_joint_state_broadcaster_after_gazebo)
    ld.add_action(spawn_husky_controller_after_joint_state_broadcaster)

    return ld


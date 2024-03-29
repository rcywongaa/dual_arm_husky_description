import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml


ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Disable top plate
    os.environ['HUSKY_TOP_PLATE_ENABLED'] = 'false'
    # Get URDF via xacro
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory("dual_arm_husky_description"),
            "urdf",
            "dual_arm_husky.urdf.xacro"
        ),
        mappings={
            'name': 'husky'
        }
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = load_file(
        "husky_dual_ur_moveit_config",
        "config/husky.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml = load_yaml(
        "dual_arm_husky_description", "config/kinematics.yaml"
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("dual_arm_husky_description",
                                   "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "dual_arm_husky_description", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        'robot_description_planning':
        load_yaml('husky_dual_ur_moveit_config', 'config/joint_limits.yaml')
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
        #prefix="gdbserver localhost:3000"
    )

    # Bridge /clock
    start_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

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
        name='spawn_robot',
        arguments=['-topic', 'robot_description',
                   '-z', '0.15',
                   '-unpause'],
        output='screen',
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        # parameters=[{"use_sim_time": True}]
    )

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    spawn_left_arm_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_joint_trajectory_controller', '-c', '/controller_manager'
        ],
        output='screen',
    )

    spawn_right_arm_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_arm_joint_trajectory_controller', '-c',
            '/controller_manager'
        ],
        output='screen',
    )

    spawn_left_arm_base_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_base_joint_trajectory_controller', '-c',
            '/controller_manager'
        ],
        output='screen',
    )

    spawn_right_arm_base_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_arm_base_joint_trajectory_controller', '-c',
            '/controller_manager'
        ],
        output='screen',
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"
        ],
    )

    # When launching with gdb, the planning interactive marker may not appear
    # due to  MoveIt not being fully up when rviz is launched.
    # Simply restart (or duplicate) the rviz MoveIt plugin to get back the interactive marker
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("dual_arm_husky_description"),
                "rviz/default.rviz"),
            # '--ros-args', '--log-level', 'DEBUG'
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
        # prefix="gdbserver localhost:3000"
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(ign_gazebo)
    ld.add_action(spawn_robot)
    # ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_robot,
                                    on_exit=[spawn_joint_state_broadcaster])))
    ld.add_action(
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[
                spawn_husky_velocity_controller,
                spawn_left_arm_joint_trajectory_controller, # Will have resource conflict if spawned with left_arm_base_joint_trajectory_controller
                # spawn_left_arm_base_joint_trajectory_controller, # husky position to velocity controller not implemented yet
                spawn_right_arm_joint_trajectory_controller
                # spawn_right_arm_base_joint_trajectory_controller # husky position to velocity controller not implemented yet
            ],
        )))
    ld.add_action(
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[run_move_group_node]
        )))
    ld.add_action(start_bridge)
    ld.add_action(rviz_node)
    ld.add_action(static_tf)

    return ld


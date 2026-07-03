"""Spawn the go1 robot (description/robot.urdf.xacro) into Gazebo Sim (gz sim)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Standing pose for the 12 leg joints (hip, thigh, calf per leg), computed from
# the 2-link leg geometry (thigh = calf = 0.213m) for a 0.30m standing height:
# thigh_angle = arccos(H / 2L), calf_angle = -2*thigh_angle. See go1.xacro's
# <ros2_control> block, which uses the same numbers as spawn-time initial values.
_STAND_POSE = [0.0, 0.789465, -1.578930] * 4  # FR, FL, RR, RL - matches
# the joint order in config/go1_controllers.yaml's position_controller.joints


# Snap-confined apps (e.g. VS Code installed as a snap) leak SNAP_*/GTK_*/
# XDG_DATA_* variables into terminals spawned from them. When present, the
# gz sim GUI's dynamic linker picks up an incompatible libpthread.so.0 from
# the core20 snap and crashes with a symbol lookup error. UnsetEnvironmentVariable
# mutates the launch context's environment (plain os.environ edits here don't
# propagate, since LaunchContext snapshots os.environ before this file runs).
_SNAP_LEAK_VARS = [
    'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_DATA', 'SNAP_USER_DATA',
    'SNAP_USER_COMMON', 'SNAP_COMMON', 'SNAP_ARCH', 'SNAP_REVISION',
    'SNAP_INSTANCE_NAME', 'SNAP_CONTEXT', 'SNAP_COOKIE', 'SNAP_REAL_HOME',
    'SNAP_EUID', 'SNAP_UID', 'SNAP_LAUNCHER_ARCH_TRIPLET', 'SNAP_VERSION',
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
    'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR',
    'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'XDG_DATA_DIRS', 'XDG_DATA_HOME',
]


def generate_launch_description():
    pkg_main_bot = get_package_share_directory('main_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = PathJoinSubstitution(
        [pkg_main_bot, 'description', 'robot.urdf.xacro']
    )
    default_world = PathJoinSubstitution(
        [pkg_main_bot, 'worlds', 'dog_test_world.sdf']
    )

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description=(
            'Gazebo world to load: a bare name resolved via GZ_SIM_RESOURCE_PATH '
            '(e.g. empty.sdf) or an absolute path. Defaults to the dog_test_world '
            'terrain course (rocks, steps, pit, slope, wave, stairs).'
        )
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='go1',
        description='Name of the entity spawned in Gazebo'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument(
        'z', default_value='0.32',
        description=(
            'Spawn height. Slightly above the 0.30m standing leg extension so the '
            'robot settles onto its feet on its own rather than clipping into the ground.'
        )
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world, ' -r']}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', x, '-y', y, '-z', z,
        ],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        # Topic list lives in config/gz_bridge.yaml instead of inline arguments -
        # keeps bridge wiring in one reviewable file next to go1_controllers.yaml.
        parameters=[{
            'config_file': os.path.join(pkg_main_bot, 'config', 'gz_bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
    )

    # gz_ros2_control's controller_manager lives inside the gz sim process (loaded
    # by the <ros2_control>/<gazebo><plugin> block in go1.xacro); these spawners
    # just activate the controllers it already knows about. `spawner` blocks until
    # the controller_manager services appear, so chaining straight off spawn_robot's
    # exit (rather than adding an arbitrary delay) is safe. --switch-timeout is
    # generous because the dog_test_world terrain course (~80 models) makes the
    # controller_manager's update loop slow to respond while the scene is still
    # loading/rendering - the default 5s switch-controller timeout isn't enough
    # (confirmed: works fine with a 5s timeout on empty.sdf, times out on the
    # terrain world).
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30'],
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['position_controller', '--switch-timeout', '30'],
    )

    command_stand_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: ' + str(_STAND_POSE) + '}',
        ],
        output='screen',
    )

    spawn_controllers_on_robot_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, position_controller_spawner],
        )
    )
    command_stand_once_controller_active = RegisterEventHandler(
        OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[command_stand_pose],
        )
    )

    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    return LaunchDescription([
        declare_world,
        declare_robot_name,
        declare_x,
        declare_y,
        declare_z,
        declare_use_sim_time,
        *unset_snap_vars,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_controllers_on_robot_spawned,
        command_stand_once_controller_active,
    ])

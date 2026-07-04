"""Spawn the go1 robot (description/robot.urdf.xacro) into Gazebo Sim (gz sim)
with the real gait controller stack (leg_pd_controller + unitree_guide_controller)
active, and optionally an RViz view of the same live simulation alongside it.

Formerly split across sim.launch.py (position_controller stand-only demo) and
walk.launch.py (the real gait stack) - merged into one file since walking is a
strict superset of standing and juggling two near-identical launch files for
"can it stand" vs "can it walk" stopped being useful once walking worked.

Does not command any pose itself - after the controllers activate, the robot
sits in its default FSM state until driven via `ros2 run unitree_guide_controller
keyboard_input` (interactive), the GUI's movement panel, or by publishing
unitree_guide_controller/msg/Inputs on /control_input directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    rviz_config = PathJoinSubstitution([pkg_main_bot, 'rviz', 'go1.rviz'])

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

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
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description=(
            'Also open RViz (showing the live simulation - same /robot_description, '
            '/tf and /joint_states the real controllers use, not a separate mock). '
            'Pass rviz:=false to skip it.'
        )
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    # gz_ros2_control's controller_manager lives inside the gz sim process (loaded
    # by the <ros2_control>/<gazebo><plugin> block in go1.xacro); these spawners
    # just activate the controllers it already knows about. `spawner` blocks until
    # the controller_manager services appear, so chaining straight off spawn_robot's
    # exit (rather than adding an arbitrary delay) is safe. --switch-timeout is
    # generous because the dog_test_world terrain course (~80 models) makes the
    # controller_manager's update loop slow to respond while the scene is still
    # loading/rendering - the default 5s switch-controller timeout isn't enough.
    #
    # Chained: leg_pd_controller must be active before unitree_guide_controller
    # activates, since the latter claims the former's exported reference interfaces
    # (command_prefix in go1_controllers.yaml) rather than hardware directly.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30'],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['imu_sensor_broadcaster', '--switch-timeout', '30'],
    )

    leg_pd_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['leg_pd_controller', '--switch-timeout', '30'],
    )

    unitree_guide_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['unitree_guide_controller', '--switch-timeout', '30'],
    )

    spawn_jsb_and_imu_on_robot_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, imu_sensor_broadcaster_spawner],
        )
    )
    spawn_leg_pd_on_imu_active = RegisterEventHandler(
        OnProcessExit(
            target_action=imu_sensor_broadcaster_spawner,
            on_exit=[leg_pd_controller_spawner],
        )
    )
    spawn_guide_on_leg_pd_active = RegisterEventHandler(
        OnProcessExit(
            target_action=leg_pd_controller_spawner,
            on_exit=[unitree_guide_controller_spawner],
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
        declare_rviz,
        *unset_snap_vars,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        rviz_node,
        spawn_robot,
        spawn_jsb_and_imu_on_robot_spawned,
        spawn_leg_pd_on_imu_active,
        spawn_guide_on_leg_pd_active,
    ])

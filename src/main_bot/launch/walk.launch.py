"""Spawn go1 with the vendored gait controllers (unitree_guide_controller +
leg_pd_controller, from legubiao/quadruped_ros2_control) instead of the
single-pose position_controller used by sim.launch.py.

Does not command any pose itself - after the controllers activate, the robot
sits in its default FSM state until driven via `ros2 run keyboard_input
keyboard_input` (interactive) or by publishing control_input_msgs/msg/Inputs
on /control_input directly (e.g. for scripted testing).
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
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# See sim.launch.py for why this is needed (VS Code snap env leaking into the
# gz sim GUI's dynamic linker).
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

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description=(
            'Gazebo world to load. Defaults to flat ground so gait behavior can be '
            'checked in isolation from the dog_test_world terrain course; pass '
            'world:=<path to main_bot>/worlds/dog_test_world.sdf once trotting works.'
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
        description='Spawn height, same margin above standing height as sim.launch.py.'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_config:=go1_controllers_walk.yaml']),
        value_type=str,
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
        # Same bridge list as sim.launch.py (/clock, /imu, foot contacts) - independent
        # of the ros2_control-level IMU/joint state access the walk controllers use.
        parameters=[{
            'config_file': os.path.join(pkg_main_bot, 'config', 'gz_bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
    )

    # Chained: leg_pd_controller must be active before unitree_guide_controller
    # activates, since the latter claims the former's exported reference interfaces
    # (command_prefix in go1_controllers_walk.yaml) rather than hardware directly.
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
        *unset_snap_vars,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb_and_imu_on_robot_spawned,
        spawn_leg_pd_on_imu_active,
        spawn_guide_on_leg_pd_active,
    ])

"""Visualize the go1 robot (description/robot.urdf.xacro) standalone in RViz2.

No Gazebo involved - robot_state_publisher builds TF from robot_description,
joint_state_publisher(_gui) drives the 12 leg joints (sliders by default so
joint limits/mesh placement can be checked by hand), and RViz displays it.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Snap-confined apps (e.g. VS Code installed as a snap) leak SNAP_*/GTK_*/
# XDG_DATA_* variables into terminals spawned from them. When present, Qt
# apps' (rviz2 here, gz sim gui elsewhere) dynamic linker picks up an
# incompatible libpthread.so.0 from the core20 snap and crashes with a
# symbol lookup error. Same fix as sim.launch.py.
_SNAP_LEAK_VARS = [
    'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_DATA', 'SNAP_USER_DATA',
    'SNAP_USER_COMMON', 'SNAP_COMMON', 'SNAP_ARCH', 'SNAP_REVISION',
    'SNAP_INSTANCE_NAME', 'SNAP_CONTEXT', 'SNAP_COOKIE', 'SNAP_REAL_HOME',
    'SNAP_EUID', 'SNAP_UID', 'SNAP_LAUNCHER_ARCH_TRIPLET', 'SNAP_VERSION',
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
    'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR',
    'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'XDG_DATA_DIRS', 'XDG_DATA_HOME',
]

# joint_state_publisher(_gui) has no physics, so without this it defaults each
# joint independently from its URDF <limit> (midpoint if the range doesn't
# straddle zero, 0 otherwise) - for this robot that comes out to thigh=0,
# calf=-1.85, which splays the legs out instead of standing. The 'zeros'
# parameter (a joint_name -> radians map, flattened by launch_ros into
# zeros.<joint> parameters) overrides that per-joint default; same standing
# angles as sim.launch.py's _STAND_POSE/go1.xacro's <ros2_control> initial
# values, computed via arccos(H / 2L) for H=0.30m, L=0.213m. With gui:=true
# the sliders still start from this pose and can be moved by hand from there.
_STAND_POSE_ZEROS = {
    'FR_hip_joint': 0.0, 'FR_thigh_joint': 0.789465, 'FR_calf_joint': -1.578930,
    'FL_hip_joint': 0.0, 'FL_thigh_joint': 0.789465, 'FL_calf_joint': -1.578930,
    'RR_hip_joint': 0.0, 'RR_thigh_joint': 0.789465, 'RR_calf_joint': -1.578930,
    'RL_hip_joint': 0.0, 'RL_thigh_joint': 0.789465, 'RL_calf_joint': -1.578930,
}


def generate_launch_description():
    pkg_main_bot = get_package_share_directory('main_bot')

    xacro_file = PathJoinSubstitution([pkg_main_bot, 'description', 'robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_main_bot, 'rviz', 'go1.rviz'])

    gui = LaunchConfiguration('gui')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show joint_state_publisher_gui sliders instead of publishing all-zero joint states'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'zeros': _STAND_POSE_ZEROS}],
        condition=IfCondition(gui),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'zeros': _STAND_POSE_ZEROS}],
        condition=UnlessCondition(gui),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    # Closing the RViz window (or it crashing) should take robot_state_publisher
    # and joint_state_publisher_gui down with it, instead of leaving them as
    # orphans - ros2 launch doesn't do this by default when only one node exits.
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([
        declare_gui,
        *unset_snap_vars,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
        shutdown_on_rviz_exit,
    ])

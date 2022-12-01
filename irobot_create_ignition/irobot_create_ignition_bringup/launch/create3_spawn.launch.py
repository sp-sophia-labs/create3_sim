import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'

ARGUMENTS = [
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Use ros_ign_bridge'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Create3 robot name'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_namespace', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to apply namespace'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():

    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    create_common_bringup_dir = os.path.join(
        pkg_irobot_create_common_bringup, 'launch')
    create_ignition_bringup_dir = os.path.join(
        pkg_irobot_create_ignition_bringup, 'launch')

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    x_dock = OffsetParser(x, 0.157)
    yaw_dock = OffsetParser(yaw, 3.1416)
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # Spawn robot
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=['-name', robot_name,
                       '-topic', [namespace, '/robot_description'],
                       '-Y', yaw,
                       '-x', x,
                       '-y', y,
                       '-z', z]),

        # Dock
        Node(package='ros_ign_gazebo',
             executable='create',
             output='screen',
             arguments=['-name', (robot_name, '_standard_dock'),
                        '-topic', [namespace, '/standard_dock_description'],
                        '-Y', yaw_dock,
                        '-x', x_dock,
                        '-y', y,
                        '-z', z]),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_common_bringup_dir, 
                    'robot_description.launch.py')),
            launch_arguments={'gazebo': 'ignition'}.items()),

        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_common_bringup_dir,
                    'dock_description.launch.py')),
            condition=IfCondition(LaunchConfiguration('spawn_dock')),
            # The robot starts docked
            launch_arguments={'x': x_dock,
                              'y': y,
                              'z': z,
                              'yaw': yaw_dock,
                              'gazebo': 'ignition'}.items()),

        # ROS Ign bridge TODO solve
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_ignition_bringup_dir,
                    'create3_ros_ignition_bridge.launch.py')),
            launch_arguments=[('world', LaunchConfiguration('world')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time')),
                              ('robot_name', robot_name),
                              ('namespace', namespace),
                              ('use_namespace', LaunchConfiguration('use_namespace'))],
            condition=IfCondition(LaunchConfiguration('bridge'))),

        # Create3 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_common_bringup_dir,
                    'create3_nodes.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_ignition_bringup_dir,
                    'create3_ignition_nodes.launch.py')),
            launch_arguments=[('robot_name', LaunchConfiguration('robot_name'))]),

        # Rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    create_common_bringup_dir,
                    'rviz2.launch.py')),
            launch_arguments=[('namespace', namespace)],
            condition=IfCondition(LaunchConfiguration('use_rviz')))
    ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(bringup_cmd_group)
    
    return ld

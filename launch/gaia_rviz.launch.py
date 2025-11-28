from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file para visualização com RViz (sem Gazebo) + controle Gaia.
    """
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_mode',
            default_value='velocity',
            description='Control mode: velocity, differential, or shim'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        )
    )
    
    # Get URDF (versão sem plugins Gazebo)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='cat')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('cylidrone_simulation'), 'urdf', 'cylidrone_rviz.urdf']
            ),
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # Joint State Publisher removido - gaia_bridge publica joint_states
    
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('cylidrone_simulation'), 'config', 'cylidrone.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    
    # Gaia Bridge Node (modo configurável via parâmetro)
    gaia_bridge_node = Node(
        package='gaia_simulation_bridge',
        executable='gaia_bridge_node',
        name='gaia_bridge_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('gaia_simulation_bridge'),
                'config',
                'gaia_params.yaml'
            ]),
            {'control_mode': LaunchConfiguration('control_mode')}
        ],
        output='screen',
    )
    
    nodes = [
        robot_state_publisher_node,
        gaia_bridge_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)

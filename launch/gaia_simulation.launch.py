from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch file para simulação com controle Gaia.
    Permite escolher o modo de controle via argumento.
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
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Launch Gazebo simulation (set to false for RViz only)'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        )
    )
    
    # Get URDF - use different URDF based on mode
    urdf_file = PythonExpression([
        "'cylidrone.urdf' if '", LaunchConfiguration('use_gazebo'), "' == 'true' else 'cylidrone_rviz.urdf'"
    ])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='cat')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('cylidrone_simulation'), 'urdf', urdf_file]
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
    
    # Gazebo (only if use_gazebo is true)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={'verbose': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )
    
    # Spawn robot in Gazebo (only if use_gazebo is true)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'cylidrone',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )
    
    # RViz (only if rviz is true and use_gazebo is false)
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
    
    # Load and start controllers (only when using Gazebo)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )
    
    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controllers', '--controller-manager', '/controller_manager'],
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )
    
    load_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controllers', '--controller-manager', '/controller_manager'],
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )
    
    # Joint state publisher for RViz mode (publishes joint states from joint commands)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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
    ]
    
    # Add conditional nodes
    nodes_to_launch = declared_arguments + nodes + [
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_position_controller,
        load_velocity_controller,
        joint_state_publisher_node,
        rviz_node,
    ]
    
    return LaunchDescription(nodes_to_launch)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.qos import QoSProfile, DurabilityPolicy

def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    # initial_value_file = LaunchConfiguration('initial_value_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    robot_ip = LaunchConfiguration('robot_ip')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    port = LaunchConfiguration('port')
    port_realtime = LaunchConfiguration('port_realtime')
    session_inactivity_timeout_ms = LaunchConfiguration('session_inactivity_timeout_ms')
    connection_inactivity_timeout_ms = LaunchConfiguration('connection_inactivity_timeout_ms')
    use_internal_bus_gripper_comm = LaunchConfiguration('use_internal_bus_gripper_comm')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    sim_ignition = LaunchConfiguration('sim_ignition')
    sim_isaac = LaunchConfiguration('sim_isaac')
    gripper_joint_name = LaunchConfiguration('gripper_joint_name')
    gripper_max_velocity = LaunchConfiguration('gripper_max_velocity')
    gripper_max_force = LaunchConfiguration('gripper_max_force')
    gazebo_renderer = LaunchConfiguration('gazebo_renderer')
    gripper_com_port = LaunchConfiguration('gripper_com_port')
    isaac_joint_commands = LaunchConfiguration('isaac_joint_commands')
    isaac_joint_states = LaunchConfiguration('isaac_joint_states')
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ), " ",
            "robot_ip:=", robot_ip, " ",
            "username:=", username, " ",
            "password:=", password, " ",
            "port:=", port, " ",
            "port_realtime:=", port_realtime, " ",
            "session_inactivity_timeout_ms:=", session_inactivity_timeout_ms, " ",
            "connection_inactivity_timeout_ms:=", connection_inactivity_timeout_ms, " ",
            "use_internal_bus_gripper_comm:=", use_internal_bus_gripper_comm, " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "fake_sensor_commands:=", fake_sensor_commands, " ",
            "sim_gazebo:=", sim_gazebo, " ",
            "sim_ignition:=", sim_ignition, " ",
            "sim_isaac:=", sim_isaac, " ",
            "gripper_joint_name:=", gripper_joint_name, " ",
            "gripper_max_velocity:=", gripper_max_velocity, " ",
            "gripper_max_force:=", gripper_max_force, " ",
            "gazebo_renderer:=", gazebo_renderer, " ",
            "gripper_com_port:=", gripper_com_port, " ",
            "simulation_controllers:=", controllers_file, " ",
            "isaac_joint_commands:=", isaac_joint_commands, " ",
            "isaac_joint_states:=", isaac_joint_states, " ",
            # "initial_value:=", initial_value_file, " ",
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # World file path
    world_file_path = PathJoinSubstitution(
        [FindPackageShare("mobile_manipulator"), "worlds", "td3.world"]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            # robot_description,
            PathJoinSubstitution([FindPackageShare(description_package), "config", controllers_file]),
            {"use_sim_time": use_sim_time},
        ],
        output='screen',
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file_path],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_manipulator',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
            '-unpause'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_traj_cont', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_cont', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_controllers_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, joint_trajectory_controller_spawner, diff_drive_controller_spawner],
        ) 
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        gzserver,
        gzclient,
        spawn_robot,
        spawn_controllers_handler,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

def generate_launch_description():
    declared_arguments = []
    # Description package
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mobile_manipulator",
            description="Package with robot description files.",
        ),
    )
    # Description file
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="integrate_execution.xacro",
            description="Robot description file.",
        ),
    )
    # Controllers configuration file
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_control.yaml",
            description="Controllers configuration file.",
        ),
    )
    # Initial value file name
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_value_file_name",
            default_value="initial_value.yaml",
            description="Initial Value",
        ),
    )
    # Launch RViz
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
    )
    # Use sim time
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true",
        ),
    )
    # Robot Ip
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address by which the robot can be reached.",
        ),
    )
    # Username
    declared_arguments.append(
        DeclareLaunchArgument(
            "username",
            default_value="admin",
            description="Username for SSH connection.",
        ),
    )
    # Password
    declared_arguments.append(
        DeclareLaunchArgument(
            "password",
            default_value="admin",
            description="Password for SSH connection.",
        ),
    )
    # Port
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="10000",
            description="Port for SSH connection.",
        ),
    )
    # Port Realtime
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_realtime",
            default_value="10001",
            description="Port for SSH connection.",
        ),
    )
    # Session Inactivity timeout ms
    declared_arguments.append(
        DeclareLaunchArgument(
            "session_inactivity_timeout_ms",
            default_value="60000",
            description="Session Inactivity timeout ms.",
        ),
    )
    # Connection Inactivity timeout ms
    declared_arguments.append(
        DeclareLaunchArgument(
            "connection_inactivity_timeout_ms",
            default_value="2000",
            description="Connection Inactivity timeout ms.",
        ),
    )
    # Use internal bus for gripper communication
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication.",
        ),
    )
    # Use fake hardware
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for simulation.",
        ),
    )
    # Fake sensor commands
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Fake sensor commands for simulation.",
        ),
    )
    # Use Gazebo simulator
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo simulator.",
        ),
    )
    # Use Ignition simulator
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Use Ignition simulator.",
        ),
    )
    # Use Isaac simulator
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Use Isaac simulator.",
        ),
    )
    # Gripper Joint name
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="right_finger_bottom_joint",
            description="Gripper Joint name.",
        ),
    )
    # Gripper Max Velocity
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Gripper Max Velocity.",
        ),
    )
    # Gripper Max Force
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Gripper Max Force.",
        ),
    )
    # Gazebo Renderer
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_renderer",
            default_value="ogre2",
            description="Gazebo Renderer.",
        ),
    )
    # Gripper com port
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_com_port",
            default_value="/dev/ttyUSB0",
            description="Use gripper commands from Isaac.",
        ),
    )
    # Isaac joint commands
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac_joint_commands",
            default_value="/isaac_joint_commands",
            description="Use joint commands from Isaac.",
        ),
    )
    # Isaac joint states
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac_joint_states",
            default_value="/isaac_joint_states",
            description="Use joint states from Isaac.",
        ),
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

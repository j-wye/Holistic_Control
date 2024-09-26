from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    
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
    
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
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
            "isaac_joint_states:=", isaac_joint_states,
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view.rviz"]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        # output="screen",
    )

    delayed_rviz_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        # output="screen",
    )

    twist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
        # output="screen",
    )
    
    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        # output="screen",
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
        # output="screen",
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delayed_rviz_node,
        joint_trajectory_controller_spawner,
        twist_controller_spawner,
        robotiq_gripper_controller_spawner,
        diff_drive_controller_spawner,
    ]

    return nodes_to_start


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
            default_value="false",
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
            default_value="ogre",
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
    
    
    # Launch RViz
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

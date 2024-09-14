from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    #robot type
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Want to See which model?",
            choices=["arm", "base", "integrate"],
            default_value="integrate",
        )
    )
    #robot ip
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="Robot's IP address",
            default_value="xxx.yyy.zzz.www",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mobile_manipulator"), "urdf", "execution.urdf.xacro"]
            )," ",
            "robot_type:=", robot_type, " ",
            "robot_ip:=", robot_ip, " ",
            "name:=mobile_manipulator", " ",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mobile_manipulator"), "rviz", "integrated_model.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

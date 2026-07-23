import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, EnvironmentVariable
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gesture_gazebo_arm")

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "gesture_arm.urdf.xacro"
    ])

    controllers_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "controllers.yaml"
    ])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "controllers_file:=",
            controllers_file
        ])
    }

    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            "/opt/ros/jazzy/lib:",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "gesture_arm",
            "-z", "0.05",
        ],
        output="screen"
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
        output="screen"
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
        output="screen"
    )

    delayed_spawn_robot = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    delayed_controllers = TimerAction(
        period=12.0,
        actions=[
            spawn_joint_state_broadcaster,
            spawn_arm_controller
        ]
    )

    return LaunchDescription([
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        delayed_spawn_robot,
        delayed_controllers
    ])
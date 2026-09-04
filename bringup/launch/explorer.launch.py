from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_simulation = LaunchConfiguration("use_simulation")
    joystick_config_file = LaunchConfiguration("joystick_config_file")
    publish_ee_pose_from_tf = LaunchConfiguration("publish_ee_pose_from_tf")

    use_actuator_interface = PythonExpression([
        "'false' if '", use_simulation, "' == 'true' else 'true'"
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "use_simulation",
            default_value="false",
            description="Whether to launch the Gazebo simulation environment.",
        ),
        DeclareLaunchArgument(
            "joystick_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("joystick_mapper"),
                "config",
                "joystick_3d.yaml",
            ]),
            description="Joystick mapper parameter file.",
        ),
        DeclareLaunchArgument(
            "publish_ee_pose_from_tf",
            default_value="true",
            description="Publish /ee_pose from TF until qontrol_controller provides it.",
        ),
    ]

    controller_config = PathJoinSubstitution([
        FindPackageShare("cartesian_manager"),
        "config",
        "explorer_params.yaml",
    ])

    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"),
            "/launch/simulation_base.launch.py",
        ]),
        launch_arguments={
            "use_POC2": "true",
            "gui": gui,
            "use_sim_time": use_simulation,
            "rviz_delay": "3.0",
            "extra_controllers_config": controller_config,
            "use_custom_controllers": "true",
        }.items(),
        condition=IfCondition(use_simulation),
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"),
            "/launch/hardware_base.launch.py",
        ]),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_simulation,
            "use_actuator_interface": use_actuator_interface,
            "can_port": "can0",
            "host_id": "45",
            "use_POC2": "true",
            "rviz_delay": "3.0",
            "extra_controllers_config": controller_config,
            "use_custom_controllers": "true",
        }.items(),
        condition=UnlessCondition(use_simulation),
    )

    spawner_qontrol = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
    )

    spawner_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    manager_node = Node(
        package="cartesian_manager",
        executable="cartesian_manager_node",
        name="cartesian_manager",
        output="screen",
        parameters=[controller_config],
    )

    tf_pose_publisher_node = Node(
        package="cartesian_manager",
        executable="tf_pose_publisher_node",
        name="tf_pose_publisher",
        output="screen",
        parameters=[controller_config, {"use_sim_time": use_simulation}],
        condition=IfCondition(publish_ee_pose_from_tf),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    joystick_mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("joystick_mapper"),
                "launch",
                "joystick_mapper.launch.py",
            ])
        ),
        launch_arguments={
            "config_file": joystick_config_file,
        }.items(),
    )

    delayed_spawner_qontrol = TimerAction(
        period=2.0,
        actions=[spawner_qontrol],
    )

    return LaunchDescription(declared_arguments + [
        robot_simulation,
        robot_hardware,
        delayed_spawner_qontrol,
        spawner_gripper_controller,
        tf_pose_publisher_node,
        manager_node,
        joy_node,
        joystick_mapper_launch,
    ])

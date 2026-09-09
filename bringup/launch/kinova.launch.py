from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    use_fake_hardware = PythonExpression([
        "'true' if '", use_simulation, "' == 'true' else 'false'"
    ])
    fake_sensor_commands = PythonExpression([
        "'true' if '", use_simulation, "' == 'true' else 'false'"
    ])
    
    robot_ip = LaunchConfiguration("robot_ip")

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
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address by which the robot can be reached."
        ),
        DeclareLaunchArgument(
            "publish_ee_pose_from_tf",
            default_value="true",
            description="Publish /ee_pose from TF until qontrol_controller provides it.",
        ),
    ]

    urdf_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kortex_description"), "robots", "gen3.xacro"]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "arm",
            " ",
            "arm:=",
            "gen3",
            " ",
            "dof:=7",
            " ",
            "prefix:=",
            "",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "gripper:=",
            "robotiq_2f_85",
            " ",
            "use_internal_bus_gripper_comm:=",
            "true",
            " ",
            "gripper_max_velocity:=",
            "100.0",
            " ",
            "gripper_max_force:=",
            "100.0",
            " ",
            "gripper_joint_name:=",
            "robotiq_85_left_knuckle_joint",
            " ",
            "use_external_cable:=",
            "true",
        ]
    )

    robot_description = {"robot_description": urdf_cmd}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    controller_config = PathJoinSubstitution([
        FindPackageShare("cartesian_manager"),
        "config",
        "kinova_params.yaml"
    ])

    # --------------------------------------------------------------------------
    # Controllers spawner
    # --------------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    spawner_qontrol = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--activate",
        ],
        output="screen",
    )

    # Spawner for robotiq_gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Spawner for fault_controller
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Other Nodes
    # --------------------------------------------------------------------------
  
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

    # --------------------------------------------------------------------------
    # Event Handlers
    # --------------------------------------------------------------------------
    delayed_spawner_qontrol = TimerAction(
        period=2.0,
        actions=[spawner_qontrol]
    )

    # --------------------------------------------------------------------------
    # Launch Description
    # --------------------------------------------------------------------------

    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delayed_spawner_qontrol,
        gripper_controller_spawner,
        manager_node,
        joy_node,
        joystick_mapper_launch,
        tf_pose_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
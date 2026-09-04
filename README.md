# cartesian_manager

`cartesian_manager` is a ROS 2 package that sits between user-level Cartesian command sources and the downstream `qontrol_controller`.

It has two jobs:

1. Collect Cartesian velocity inputs, such as joystick and visual-servoing commands.
2. Apply the selected command mode, then publish either:
   - a shaped Cartesian velocity command for QP Cartesian control, or
   - a named joint-position target command for QP joint-target control.

The package deliberately keeps robot control authority in `qontrol_controller`. In particular, joint targets are not executed directly here. `cartesian_manager` only selects and publishes the target. `qontrol_controller` converts the joint error to a bounded joint velocity task inside its QP.

## Package Layout

```text
cartesian_manager/
  CMakeLists.txt
  package.xml
  README.md
  bringup/
    config/
      explorer_params.yaml
    launch/
      explorer.launch.py
  docs/
    technical_guide.md
  include/cartesian_manager/
    core/
      input_manager.hpp
      manager.hpp
      types.hpp
      shapers/
        shaper.hpp
        geometric/
          jaco.hpp
          snake.hpp
        behaviour/
          joint_target.hpp
    ros/
      cartesian_manager.hpp
      parameter_parsing.hpp
      tf_pose_publisher.hpp
      topic_manager.hpp
  src/
    cartesian_manager.yaml
    core/
      input_manager.cpp
      manager.cpp
      shapers/geometric/
        jaco.cpp
        snake.cpp
    ros/
      cartesian_manager.cpp
      main.cpp
      parameter_parsing.cpp
      tf_pose_publisher.cpp
      tf_pose_publisher_main.cpp
      topic_manager.cpp
```

## Runtime Flow

Normal Cartesian command flow:

```text
joystick CartesianVelocityCommand / visual-servoing TwistStamped
        |
        v
CartesianManagerROS subscribers
        |
        v
manager_core::InputManager
        |
        v
manager_core::Manager
        |
        v
geometric shaper: both, jaco, or snake
        |
        v
/cartesian_command TwistStamped
        |
        v
qontrol_controller CartesianVelocity QP task
```

Joint target flow:

```text
/mode_request: "behaviour/joint_target/home"
        |
        v
manager_core::Manager validates "home"
        |
        v
CartesianManagerROS publishes /joint_target_command sensor_msgs/JointState once
        |
        v
qontrol_controller JointVelocity QP task moves toward the target
        |
        v
cartesian_manager immediately returns to passthrough
```

This one-shot dispatch is important. `cartesian_manager` must not stay in `joint_target` mode forever, otherwise it would keep publishing zero Cartesian velocity and joystick control would never resume.

## Build

From the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select cartesian_manager
```

For local CMake-only validation from this package directory:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cmake -S . -B /tmp/cartesian_manager_build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/cartesian_manager_build --target cartesian_manager_node
```

`/usr/bin/python3` is recommended when generating parameters if another Python environment does not provide ROS Python packages such as `catkin_pkg`.

## Launch

The Explorer-style bringup is:

```bash
source install/setup.bash
ros2 launch cartesian_manager explorer.launch.py
```

Simulation:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true
```

Hardware:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=false
```

The launch file starts:

- the Explorer simulation or hardware base launch,
- `qontrol_explorer`,
- the gripper controller,
- `tf_pose_publisher_node`, which temporarily publishes `/ee_pose` from TF,
- `cartesian_manager_node`,
- `joy_node`,
- `joystick_mapper`.

The launch/config files are installed from:

- `bringup/launch/explorer.launch.py`
- `bringup/config/explorer_params.yaml`

## Important Topics

Default topics from `bringup/config/explorer_params.yaml`:

| Topic | Type | Direction | Meaning |
| --- | --- | --- | --- |
| `/joystick_cartesian_command` | `extender_msgs/msg/CartesianVelocityCommand` | input | Joystick Cartesian velocity command with an angular orientation-frame selector. |
| `/visual_servoing_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Visual-servoing Cartesian velocity command. |
| `/mode_request` | `std_msgs/msg/String` | input | Mode selection request. |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` | input | Current end-effector pose. The Explorer launch currently supplies it from TF; Qontrol can publish the same interface later. |
| `/ee_velocity` | `geometry_msgs/msg/TwistStamped` | input | Current end-effector velocity from `qontrol_controller`. |
| `/ee_jac` | `std_msgs/msg/Float64MultiArray` | input | Current end-effector Jacobian. |
| `/joint_states` | `sensor_msgs/msg/JointState` | input | Current joint state. |
| `/cartesian_command` | `geometry_msgs/msg/TwistStamped` | output | Cartesian velocity sent to `qontrol_controller`. |
| `/joint_target_command` | `sensor_msgs/msg/JointState` | output | Named joint-position target sent to `qontrol_controller`. |

## Mode Requests

Publish mode requests as `std_msgs/msg/String` on `/mode_request`.

Examples:

```bash
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/both'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/jaco'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/snake'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/passthrough'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/joint_target/home'}"
```

Mode strings are normalized before parsing:

- uppercase becomes lowercase,
- `-` becomes `_`,
- the package uses the British spelling `behaviour`.

## Parameters

Generated parameter definitions live in:

- `src/cartesian_manager.yaml`

Runtime Explorer parameters live in:

- `bringup/config/explorer_params.yaml`

The main groups are:

- `update_rate_hz`
- `output_frame_id`
- `default_input_frame_id`
- `hybrid_frame_cone_angle_deg`
- `topics`
- `inputs`
- `shapers`
- `behaviours`

The node validates runtime parameter updates. Invalid updates are rejected by the ROS parameter callback before they are applied internally.

The independent `tf_pose_publisher_node` parameters are:

- `base_frame_id` (default `base_link`)
- `effector_frame_id` (default `tool0`)
- `output_topic` (default `/ee_pose`)
- `publish_rate_hz` (default `100.0`)

It looks up the latest `base_frame_id <- effector_frame_id` transform and publishes it as a
normalized `PoseStamped`. Missing or invalid transforms are skipped with a throttled warning.

The Explorer launch enables this temporary adapter by default. Once Qontrol publishes `/ee_pose`,
disable the adapter so that only one node publishes the pose:

```bash
ros2 launch cartesian_manager explorer.launch.py publish_ee_pose_from_tf:=false
```

## Frames

Input Cartesian commands must already be in `default_input_frame_id`.

Cartesian Manager itself does not perform TF conversion. If an input message has an empty
`header.frame_id`, it is treated as `default_input_frame_id`. If it has a different frame, the
command is rejected. The standalone TF pose adapter only supplies the `/ee_pose` input and remains
independent of command conversion.

Joystick commands additionally select how their angular vector is mapped into the base frame:

- `base_frame`: use the angular vector unchanged.
- `effector_frame`: rotate it by the latest end-effector orientation.
- `hybrid_frame`: use the stateful tool-Z hybrid frame with the configured anti-singularity cone.

Linear joystick velocity is never rotated. Effector and hybrid commands are rejected until a valid
end-effector pose in `default_input_frame_id` has been received. The default hybrid cone half-angle
is 5 degrees and can be changed with `hybrid_frame_cone_angle_deg`.

## Joint Targets

Named joint targets are configured under:

```yaml
cartesian_manager:
  ros__parameters:
    behaviours:
      joint_targets:
        joint_names:
          - joint_1
          - joint_2
          - joint_3
          - joint_4
          - joint_5
          - joint_6
        target_names:
          - home
        positions:
          - 2.5
          - 0.3
          - -2.4
          - 2.97
          - 1.2
          - -0.5
```

`positions` is flattened in `target_names` order. If there are 6 joints and 2 targets, the array must contain 12 values.

When `behaviour/joint_target/home` is received, the node publishes:

- `msg.name = joint_names`
- `msg.position = positions for home`

on `/joint_target_command`.

## Relation To qontrol_controller

`qontrol_controller` is expected to subscribe to `/cartesian_command` and `/joint_target_command`.

The joint-target execution belongs there:

```text
q_error = q_target - q_current
qdot_target = joint_target_gain * q_error
qdot_target is clamped by joint_target_max_velocity
qdot_target is sent to a Qontrol Task::JointVelocity
```

This keeps joint limits and QP constraints active while moving to a target.

## Development Notes

For extension details, see:

- `docs/technical_guide.md`

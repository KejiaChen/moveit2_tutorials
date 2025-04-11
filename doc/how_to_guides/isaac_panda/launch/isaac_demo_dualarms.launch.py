import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():  
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components", # isaac or mock_components
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    use_sensone_left = DeclareLaunchArgument(
        "use_sensone_left",
        default_value="true",
        description="Whether to include the BotaSys external force torque sensor in the left robot model",
    )

    use_sensone_right = DeclareLaunchArgument(
        "use_sensone_right",
        default_value="true",
        description="Whether to include the BotaSys external force torque sensor in the right robot model",
    )

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
                "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                "use_sensone_right": LaunchConfiguration("use_sensone_right"),
            },
            )
        .robot_description_semantic(
            file_path="config/panda.srdf.xacro",
            mappings={
                "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                "use_sensone_right": LaunchConfiguration("use_sensone_right"),
            },
            )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    move_group_capabilities,
                    ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_panda_moveit_config"),
        "launch",
        "dual_demo_rviz_pose_tracking.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

     # Get parameters for the Servo node
    follow_servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/follow_pose_tracking_settings.yaml",
        )
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/follow_panda_simulated_config_pose_tracking.yaml",
        )
        .to_dict()
    )
    
    lead_servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/lead_pose_tracking_settings.yaml",
        )
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/lead_panda_simulated_config_pose_tracking.yaml",
        )
        .to_dict()
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "left_panda_link0", "right_panda_link0"],
    )

    # The servo cpp interface demo
    # Creates the follower Servo node and publishes commands to it
    follow_servo_node = Node(
        package="moveit_servo",
        executable="follow_demo",
        output="screen",
        parameters=[
            # moveit_config.to_dict(),
            follow_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )
    
    # The servo cpp interface demo
    # Creates the leader Servo node and publishes commands to it
    lead_servo_node = Node(
        package="moveit_servo",
        executable="lead_demo",
        output="screen",
        parameters=[
            # moveit_config.to_dict(),
            lead_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
    )

    # Load controllers
    # load_controllers = []
    # for controller in [
    #     "joint_state_broadcaster",
    #     "left_arm_controller",
    #     "left_hand_controller",
    #     # "right_arm_controller",
    #     # "right_hand_controller",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]

    return LaunchDescription(
        [   
            ros2_control_hardware_type,
            use_sensone_left,
            use_sensone_right,
            rviz_node,
            static_tf_node,
            follow_servo_node,
            lead_servo_node,
            ros2_control_node,
            robot_state_publisher,
            move_group_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_hand_controller_spawner,
            right_arm_controller_spawner,
            right_hand_controller_spawner,
        ]
        # + load_controllers
    )

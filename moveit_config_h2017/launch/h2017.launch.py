import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command

from srdfdom.srdf import SRDF
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_rsp_launch(ld, moveit_config):
    """Launch file for robot state publisher (rsp)"""

    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="50.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld


def generate_moveit_rviz_launch(ld, moveit_config):
    """Launch file for rviz"""

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    ld.add_action(rviz_node)

    return ld


def generate_static_virtual_joint_tfs_launch(ld, moveit_config):

    name_counter = 0

    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1
    return ld


def generate_spawn_controllers_launch(ld, moveit_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld


def generate_warehouse_db_launch(ld, moveit_config):
    """Launch file for warehouse database"""
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )
    ld.add_action(DeclareLaunchArgument("reset", default_value="false"))

    # The default DB port for moveit (not default MongoDB port to avoid potential conflicts)
    ld.add_action(DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"))

    # The default DB host for moveit
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # Load warehouse parameters
    db_parameters = [
        {
            "overwrite": False,
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]
    # Run the DB server
    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        # TODO(dlu): Figure out if this needs to be run in a specific directory
        # (ROS 1 version set cwd="ROS_HOME")
        parameters=db_parameters,
        condition=IfCondition(LaunchConfiguration("db")),
    )
    ld.add_action(db_node)

    # If we want to reset the database, run this node
    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    ld.add_action(reset_node)

    return ld


def generate_move_group_launch(ld, moveit_config):

    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true")
    )
    ld.add_action(
        DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true")
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareLaunchArgument("monitor_dynamics", default_value="false"))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params
    )
    ld.add_action(move_group_node)
    return ld


def launch_setup(context, *args, **kwargs):
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "db",
            default_value="false",
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # Generate moveit config from files in the moveit_config package
    moveit_config_builder = MoveItConfigsBuilder("h2017", package_name="moveit_config_h2017")
    moveit_config = moveit_config_builder.to_moveit_configs()
    
    ld.add_action(DeclareLaunchArgument("robot_description",
                                        default_value="",
                                        description="[Optional] Full robot description"))
    if LaunchConfiguration("robot_description").perform(context) != "":
        robot_description = {
            'robot_description': LaunchConfiguration('robot_description')
        }
    moveit_config.robot_description = robot_description

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    ld = generate_static_virtual_joint_tfs_launch(ld, moveit_config)

    # Given the published joint states, publish tf for the robot links
    ld = generate_rsp_launch(ld, moveit_config)

    # Include the launch file for move_group
    ld = generate_move_group_launch(ld, moveit_config)

    # Run Rviz and load the default config to see the state of the move_group node
    ld = generate_moveit_rviz_launch(ld, moveit_config)

    # Include the launch file for warehouse database if enabled
    ld = generate_warehouse_db_launch(ld, moveit_config)

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld = generate_spawn_controllers_launch(ld, moveit_config)
    return ld.entities


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )

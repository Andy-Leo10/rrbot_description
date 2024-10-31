import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import (Node, SetParameter)
import xacro

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    package_description = "rrbot_description"
    package_directory = os.path.join(get_package_share_directory(package_description))

    # Load URDF File #
    # urdf_file = 'test.xacro.urdf'
    urdf_file = 'rrbot_and_control.xacro'
    robot_desc_path = os.path.join(package_directory,'urdf',urdf_file)

    # doc = xacro.parse(open(robot_desc_path))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'use_sim_time': True, 
              'robot_description': Command(['xacro ', robot_desc_path])}    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    ) 

    # Spawn the Robot #
    declare_spawn_model_name = DeclareLaunchArgument("model_name", default_value="my_robot",
                                                    description="Model Spawn Name")    
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5",
                                            description="Model Spawn Z Axis Value")
    node_ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    ) 
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )    

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )       

    # ROS-Gazebo Bridge #
    node_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge_node",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
        remappings=[
            # there are no remappings for this robot description
            #("/topic1", "/namespace/topic1"),
        ],
        output="screen",
    )        

    return LaunchDescription([
        # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
        SetParameter(name="use_sim_time", value=True),
        
        node_robot_state_publisher,
        
        declare_spawn_model_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,          
        node_ignition_spawn_entity,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        
        node_ign_bridge,
    ])

# https://githubhelp.com/ignitionrobotics/ign_ros2_control  
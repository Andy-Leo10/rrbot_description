from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_description = 'rrbot_description'  # Replace with your package name
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    pkg_models_path = os.path.join(package_directory, "models")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, pkg_models_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))    
    
    # Declare the launch argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file to load'
    )

    package_directory = FindPackageShare(package_description).find(package_description)
    
    # Use the launch argument
    world_config = LaunchConfiguration('world')
    
    # Declare GazeboSim Launch #
    gzsim_pkg = FindPackageShare("ros_ign_gazebo").find("ros_ign_gazebo")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "ign_gazebo.launch.py"])
        ),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r '),
                TextSubstitution(text='-v 1 '),
                PathJoinSubstitution([package_directory, 'worlds', world_config])
            ],
            'gz_version': '6'  # specify the version of Gazebo
        }.items()
    )

    return LaunchDescription([
        # Sets use_sim_time for all nodes started (except ignition gazebo nodes) #
        SetParameter(name="use_sim_time", value=True),
        
        declare_world_arg,
        LogInfo(msg=['---------------->         Using SDF world: ', world_config]),
        gz_sim,
    ])
    
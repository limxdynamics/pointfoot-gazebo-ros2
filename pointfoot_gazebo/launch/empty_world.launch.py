from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
import os
import sys

def generate_launch_description():
    robot_type = os.getenv("ROBOT_TYPE")

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)

    # Get URDF file path
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "pointfoot/"+ robot_type +"/xacro", "robot.xacro"]
    )

    # Load the URDF file
    robot_description = Command(["xacro ", urdf_file, " ",
            "use_gazebo:=",
            'true',
            " ",
            "use_support:=",
            'false',
            " ",])

    # Get world file path
    world_file = os.path.join(
        FindPackageShare('pointfoot_gazebo').find('pointfoot_gazebo'),
        'worlds',
        'empty_world.world'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
            ]),
            launch_arguments={
                'world': world_file, 
                'pause': 'false',
                'verbose': 'false'
            }.items(),
            
        ),

        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=['-topic', 'robot_description', '-entity', 'pointfoot_entity', 
                      '-x', '0', '-y', '0', '-z', '0.62'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])


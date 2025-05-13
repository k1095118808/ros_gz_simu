# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    # pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo') # Not used in this file
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim') # Not used in this file

    # Load the URDF file from "description" package
    # Note: Variable name was sdf_file, but it loads a .urdf file. Renamed for clarity.
    urdf_file  =  os.path.join(pkg_project_description, 'models', 'explorer_ds1', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz.'
    )

    # Visualize in RViz
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2', # It's good practice to name your nodes
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'explorer.rviz')],
       parameters=[{'use_sim_time': True}], # Added use_sim_time
       condition=IfCondition(LaunchConfiguration('rviz')),
       output='screen' # Added for better debugging
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge', # It's good practice to name your nodes
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'explorer_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True # Added use_sim_time
        }],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True}, # Correctly set
            {'robot_description': robot_desc},
            {'frame_prefix': 'explorer_ds1/'}, # Ensures TF frames are explorer_ds1/base_link etc.
            # {'publish_frequency': 120.0} # Default is 50.0, 120 is high but can be kept if needed.
        ]
    )

    # Node to convert PoseStamped to TF transform (world -> base_link)
    pose_to_tf_node = Node(
        package='ros_gz_example_bringup', # Replace with the package containing your script
        executable='pose_to_tf_publisher.py', # The Python script
        name='explorer_ds1_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, # Correctly set
            'world_frame': 'explorer_world',  # Or 'odom', consistent with RViz fixed frame
            'child_frame': 'explorer_ds1/base_link' # base_link from your URDF with prefix
        }],
        remappings=[ # Ensure it subscribes to the bridged topic
            ('/explorer_ds1/pose', '/explorer_ds1/pose') # This remapping is redundant if topics match
        ]
    )

    # A solution to fix the lidar frame: static transform publisher
    static_transform_lidar_fix_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_laser_frame_broadcaster',
        output='screen', # Added for better debugging
        parameters=[{'use_sim_time': True}], # Added use_sim_time
        arguments=[
            # Arguments: x y z yaw pitch roll parent_frame_id child_frame_id (ROS 2 Foxy+)
            # or x y z qx qy qz qw parent_frame_id child_frame_id
            # Assuming x y z qx qy qz qw variant for broader compatibility
            '0', '0', '0',  # x, y, z offset (no offset)
            '0', '0', '0', '1', # qx, qy, qz, qw rotation (no rotation - identity quaternion)
            'explorer_ds1/front_laser',          # Parent Frame (from URDF via robot_state_publisher)
            'explorer_ds1/base_link/front_laser' # Child Frame (used in Gazebo Lidar messages)
        ]
    )

    return LaunchDescription([
        declare_rviz_arg,
        bridge_node,
        rviz_node,
        robot_state_publisher_node,
        # pose_to_tf_node,
        static_transform_lidar_fix_node
    ])

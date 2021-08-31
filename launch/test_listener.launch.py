import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from math import pi

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################

    nodes = []
    # Start the actual move_group interface node
    tf_listener = Node(name='tf_listener',
                            package='rqt_tf_tools',
                            executable='tf_listener',
                            output='screen',
                            )
    # nodes.append(tf_listener)

    rqt = Node(name='rqt',
               package='rqt_gui',
               executable='rqt_gui',
               output='screen',
               arguments=['--force-discover']
               )
    nodes.append(rqt)

    static_tf_1 = Node(package='tf2_ros',
                       executable='static_transform_publisher',
                       name='tf_pub_one',
                       output='log',
                       arguments=['-0.275', '0.3', '1.478', '0.0', '0.0', str(pi), 'world', 'one'])
    nodes.append(static_tf_1)

    static_tf_2 = Node(package='tf2_ros',
                       executable='static_transform_publisher',
                       name='tf_pub_two',
                       output='log',
                       arguments=['0.425', '0.2', '0.2', str(pi/2), '0.0', str(pi), 'world', 'two'])
    nodes.append(static_tf_2)

    static_tf_3 = Node(package='tf2_ros',
                       executable='static_transform_publisher',
                       name='tf_pub_three',
                       output='log',
                       arguments=['-0.275', '0.3', '1.478', '0.0', '0.0', str(pi), 'world', 'three'])
    nodes.append(static_tf_3)

    static_tf_4 = Node(package='tf2_ros',
                       executable='static_transform_publisher',
                       name='tf_pub_four',
                       output='log',
                       arguments=['0.5', '0.1', '-2.2', str(-pi/3), str(pi/4), '0.0', 'world', 'four'])
    nodes.append(static_tf_4)
    return LaunchDescription(nodes)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        namespace='teleop_twist_keyboard',
        executable='teleop_twist_keyboard'
    )

    gps_node = Node(
        package='gps_package',
        namespace='gps',
        executable='gps_node'
    )

    simulator_control_node = Node(
        package='simulator_control',
        namespace='simulator_control',
        executable='simulator_control_node'
    )

    realsense_obj_det_node = Node(
        package='realsense_obj_det',
        namespace='realsense_obj_det',
        executable='realsense_obj_det_node'
    )    

    web_video_server_node = Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch/rs_launch.py'
                ])
            ])              
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne'),
                    'launch/velodyne-all-nodes-VLP16-launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('twist_mux'),
                    'launch/twist_mux_launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('image_publisher'),
                    'launch/image_publisher_mono.launch.py'
                ])
            ])
        ),
        gps_node,
        simulator_control_node,
        realsense_obj_det_node,
        web_video_server_node
    ])
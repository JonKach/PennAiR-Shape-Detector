from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():
    source = LaunchConfiguration('source')
    frame_rate = LaunchConfiguration('frame_rate')
    loop = LaunchConfiguration('loop')

    video_path = PathJoinSubstitution([
        FindPackageShare('shape_detection'),
        'dynamic.mp4'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value=video_path, description='video file path'),
        DeclareLaunchArgument('frame_rate', default_value='5.0'),
        DeclareLaunchArgument('loop', default_value='true'),

        Node(
            package='shape_detection',
            executable='video_publisher',
            name='video_publisher',
            parameters=[{
                'source': source,
                'frame_rate': frame_rate,
                'loop': loop,
            }],
            output='screen'
        ),
        Node(
            package='shape_detection',
            executable='shape_detector',
            name='shape_detector',
            output='screen'
        ),
    ])

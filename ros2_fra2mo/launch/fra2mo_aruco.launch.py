"""CW3 Task 4: detect all the ArUco markers in view and express their poses in the map frame.

Start after gazebo_fra2mo.launch.py and fra2mo_navigation.launch.py (AMCL must publish map -> odom).
Outputs: /aruco_marker_publisher/markers_list (ids), /aruco_marker_publisher/markers (poses in map),
         /aruco_marker_publisher/result (image with the detections drawn on it).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    marker_size = LaunchConfiguration('marker_size')
    reference_frame = LaunchConfiguration('reference_frame')

    return LaunchDescription([
        # Side of the black square: the texture ar_tag_<id>.png is black up to its edge and
        # fills the 0.30 m tag visual (models/marker_<id>/marker_<id>.sdf); the 0.32 m white
        # support behind it gives the 1 cm quiet zone.
        DeclareLaunchArgument('marker_size', default_value='0.30'),
        DeclareLaunchArgument('reference_frame', default_value='map'),

        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            parameters=[{
                'marker_size': marker_size,
                'reference_frame': reference_frame,
                'camera_frame': 'd435_optical_frame',   # optical frame of the on-board camera
                'image_is_rectified': True,             # simulated pinhole camera, no distortion
                'use_sim_time': True,
            }],
            remappings=[
                ('/image', '/camera/image'),
                ('/camera_info', '/camera/camera_info'),
            ],
            output='screen',
        ),
    ])

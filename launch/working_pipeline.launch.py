from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Driver Node (publishes /dual_fisheye/image/compressed_stream)
        Node(
            package='insta360_ros_driver',
            executable='insta360_ros_driver',
            name='insta360_ros_driver',
            output='screen'
        ),
        # 2. Topic Unwrapper Node (unwraps CameraCompressedStreamData.data -> CompressedImage)
        Node(
            package='topic_tools',
            executable='transform',
            name='stream_data_unwrapper',
            arguments=[
                '/dual_fisheye/image/compressed_stream',
                '/dual_fisheye/image/compressed',
                'sensor_msgs/msg/CompressedImage',
                'm.data'
            ],
            output='screen'
        ),
        # 3. FFmpeg Decoder Node (decompresses /dual_fisheye/image/compressed)
        Node(
            package='insta360_ros_driver',
            executable='ffmpeg_decoder',
            name='ffmpeg_decoder',
            output='screen'
        ),
        # 4. CPU OpenCV Stitcher Node (stitches /dual_fisheye/image -> /equirectangular/image)
        Node(
            package='insta360_ros_driver',
            executable='dual_fisheye2equirectangular_cpp',
            name='dual_fisheye2equirectangular_node',
            output='screen'
        )
    ])

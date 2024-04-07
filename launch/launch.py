from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{'video_device': '/dev/video11'}] 
        ),
       
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            
        )
    ])
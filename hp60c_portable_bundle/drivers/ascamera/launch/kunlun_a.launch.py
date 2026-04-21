from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ascamera_node = Node(
        namespace= "ascamera_kunlunA",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"depth_width": -1},
            {"depth_height": -1},
            {"peak_width": -1},
            {"peak_height": -1},
            {"fps": -1},
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "./ascamera/configurationfiles"},
            {"color_pcl": False},
        ],
        remappings=[]
    )

    return LaunchDescription([ascamera_node])


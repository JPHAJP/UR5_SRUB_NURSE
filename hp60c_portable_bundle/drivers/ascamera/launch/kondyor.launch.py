from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ascamera_node = Node(
        namespace= "ascamera_kondyor",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "./ascamera/configurationfiles"},
            {"color_pcl": False},
        ],
        remappings=[]
    )

    return LaunchDescription([ascamera_node])


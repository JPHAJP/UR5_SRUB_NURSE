from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ascamera_node = Node(
        namespace= "ascamera_kunlunC",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"depth_width": -1},
            {"depth_height": -1},
            {"peak_width": -1},
            {"peak_height": -1},
            {"rgb_width": -1},
            {"rgb_height": -1},
            {"fps": -1},
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "./ascamera/configurationfiles"},
            {"color_pcl": False},
        ],
        remappings=[]
    )

    return LaunchDescription([ascamera_node])

# # if launch double nodes
# def generate_launch_description():
#     ld = LaunchDescription()
#     ascamera_node = Node(
#         namespace= "ascamera_kunlunC1",
#         package='ascamera',
#         executable='ascamera_node',
#         respawn=True,
#         output='both',
#         parameters=[
#             {"depth_width": -1},
#             {"depth_height": -1},
#             {"peak_width": -1},
#             {"peak_height": -1},
#             {"rgb_width": -1},
#             {"rgb_height": -1},
#             {"fps": -1},
#             {"usb_bus_no": 3},  replace your special bus_no and path
#             {"usb_path": "2.2"},
#             {"confiPath": "./ascamera/configurationfiles"},
#             {"color_pcl": False},
#         ],
#         remappings=[]
#     )

#     ascamera_node2 = Node(
#         namespace= "ascamera_kunlunC2",
#         package='ascamera',
#         executable='ascamera_node',
#         respawn=True,
#         output='both',
#         parameters=[
#             {"depth_width": -1},
#             {"depth_height": -1},
#             {"peak_width": -1},
#             {"peak_height": -1},
#             {"rgb_width": -1},
#             {"rgb_height": -1},
#             {"fps": -1},
#             {"usb_bus_no": 3},    replace your special bus_no and path
#             {"usb_path": "2.1"},
#             {"confiPath": "./ascamera/configurationfiles"},
#             {"color_pcl": False},
#         ],
#         remappings=[]
#     )

#     ld.add_action(ascamera_node)
#     ld.add_action(ascamera_node2)

#     return ld

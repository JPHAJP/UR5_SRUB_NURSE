from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "ascamera_hp60cn",
            package='ascamera',
            executable='ascamera_node',
            respawn=True,
            output='both',
            parameters=[
                {"confiPath": "./ascamera/configurationfiles"},
				{"pub_tfTree": True},
                {"color_pcl": False}
            ]
        ),
    ])

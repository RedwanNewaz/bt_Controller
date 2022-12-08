import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    create3_controller = get_package_share_directory("create3_controller")
    param_file = os.path.join(create3_controller, "config", "dwa_param.yaml")
    dwa_node = Node(
        name='create3_controller',
        package="create3_controller",
        executable="create3_controller_node",
        arguments=[param_file]
    )


    create3_localization = get_package_share_directory("create3_localization")
    sensor_fusion = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            create3_localization + '/launch/apriltag_launch.py'))

    # return launch.LaunchDescription([robot_frame, viz_node, odom_transform])
    return launch.LaunchDescription([sensor_fusion, dwa_node])

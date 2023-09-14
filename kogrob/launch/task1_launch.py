from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("kogrob")
    #rviz_path = '/home/martin/.rviz2/config.rviz'
    rviz_path = '/home/martin/ros2_ws/src/kogrob/rviz/config.rviz'

    body_pub = Node(package='kogrob', executable='body_publisher_node')
    body_tf_br = Node(package='kogrob', executable='body_tf_subscriber_node')
    rviz = Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', str(rviz_path)])

    return LaunchDescription([body_pub, body_tf_br, rviz])

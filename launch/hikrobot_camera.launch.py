# launch/hikrobot_camera.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('hikrobot_camera').find('hikrobot_camera')
    
    param_file = LaunchConfiguration('param_file')
    declare_param_file = DeclareLaunchArgument(
        'param_file',
        default_value=[pkg_share, '/config/ros2_param.yaml'],
        description='Path to camera configuration YAML file'
    )

    camera_node = Node(
        package='hikrobot_camera',
        executable='hikrobot_camera',
        name='hikrobot_camera',
        output='screen',
        respawn=True,
        parameters=[param_file]   # 加载 YAML 参数文件
    )

    return LaunchDescription([
        declare_param_file,
        camera_node
    ])
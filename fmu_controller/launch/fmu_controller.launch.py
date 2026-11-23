from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fmu_controller',       
            executable='fmu_node',  
            name='fmu_controller',       # opsiyonel: calısırken gorunecek ad
            output='screen',             # logları terminale basar
            )
          ]
        )

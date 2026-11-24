from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fmu_controller',       
            #executable='fmu_node',  
            executable='fmu_action_server',
            #name='fmu_controller',       # opsiyonel: calısırken gorunecek ad
            name='fmu_action_server_node',
            output='screen',             # logları terminale basar
            )
          ]
        )

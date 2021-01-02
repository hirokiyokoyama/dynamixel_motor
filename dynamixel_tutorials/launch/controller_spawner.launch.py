import launch
import launch_ros.actions

import os
from ament_index_python import get_package_share_directory
param_file = os.path.join(
    get_package_share_directory('dynamixel_controllers'),
    'config',
    'dynamixel_joint_controllers.yaml'
)

spawner = launch_ros.actions.Node(
    package = 'dynamixel_controllers',
    node_executable = 'controller_spawner',
    output = 'screen',
    node_name = 'dynamixel_controller_spawner',
    parameters = [param_file],
    arguments = [
        '--manager=dxl_manager',
        '--port=pan_tilt_port',
        '--type=simple',
        'pan_controller',
        'tilt_controller'
    ]
)

def generate_launch_description():
    return launch.LaunchDescription([
        spawner
    ])

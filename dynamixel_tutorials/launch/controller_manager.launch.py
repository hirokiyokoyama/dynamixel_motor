import launch
import launch_ros.actions

manager = launch_ros.actions.Node(
    package = 'dynamixel_controllers',
    node_executable = 'controller_manager',
    output = 'screen',
    node_name = 'dynamixel_manager',
    parameters = [{
        'namespace': 'dxl_manager',
        'serial_ports': {
            'pan_tilt_port': {
                'port_name': '/dev/ttyUSB0',
                'baud_rate': 1000000,
                'min_motor_id': 1,
                'max_motor_id': 25,
                'update_rate': 20
            }
        }
    }]
)

def generate_launch_description():
    return launch.LaunchDescription([
        manager
    ])

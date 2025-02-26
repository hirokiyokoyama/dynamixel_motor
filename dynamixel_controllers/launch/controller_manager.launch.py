"""
    Start Dynamixel controller manager
    'namespace' is the name of this manager (it has to be a valid ROS graph name).
    'serial_ports' contains a map of serial ports with all their configuration paramters.
                   It is possible to specify multiple ports for a single controller_manager.
                   Each port name also has to be a valid ROS graph name.
    Once the controller manager start up it will provide serices to start, stop and restart
    dynamixel controllers, e.g. the manager below with advertise the following:
    1. /dynamixel_controller_manager/dxl_tty1/start_controller
    2. /dynamixel_controller_manager/dxl_tty1/stop_controller
    3. /dynamixel_controller_manager/dxl_tty1/restart_controller
    If the second serial port is uncommented, this manager will provide the same three
    services but with different path, e.g. for the configuration below:
    4. /dynamixel_controller_manager/dxl_tty2/start_controller
    5. /dynamixel_controller_manager/dxl_tty2/stop_controller
    6. /dynamixel_controller_manager/dxl_tty2/restart_controller
"""
import launch
import launch_ros.actions

manager = launch_ros.actions.Node(
    package = 'dynamixel_controllers',
    node_executable = 'controller_manager.py',
    output = 'screen',
    node_name = 'dynamixel_manager',
    parameters = [{
        'namespace': 'dynamixel_controller_manager',
        'serial_ports': {
            'dxl_tty1': {
                'port_name': '/dev/ttyUSB0',
                'baud_rate': 1000000,
                'min_motor_id': 1,
                'max_motor_id': 30,
                'update_rate': 10 
            }
        }
    }] 
)

def generate_launch_description():
    return launch.LaunchDescription([
        manager
    ])

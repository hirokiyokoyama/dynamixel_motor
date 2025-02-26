#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Antons Rebguns, Cody Jorgensen, Cara Slutter
#               2010-2011, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns, Cody Jorgensen, Cara Slutter'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns, Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


from threading import Thread, Lock

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from dynamixel_driver.dynamixel_serial_proxy import SerialProxy

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController

class ControllerManager(Node):
    def __init__(self):
        super().__init__(
            'dynamixel_controller_manager',
            automatically_declare_parameters_from_overrides = True)
        
        self.waiting_meta_controllers = []
        self.controllers = {}
        self.serial_proxies = {}
        self.diagnostics_rate = self.get_parameter_or(
            'diagnostics_rate', 1
        )
        if not isinstance(self.diagnostics_rate, int):
            self.diagnostics_rate = self.diagnostics_rate.value
        
        self.start_controller_lock = Lock()
        self.stop_controller_lock = Lock()

        manager_namespace = self.get_parameter(
            'namespace',
        ).value
        
        serial_ports = {}
        for name, param in self.get_parameters_by_prefix('serial_ports').items():
            name1, name2 = name.split('.')
            port = serial_ports.setdefault(name1, {})
            port[name2] = param.value

        for port_namespace, port_config in serial_ports.items():
            port_name = port_config['port_name']
            baud_rate = port_config['baud_rate']
            readback_echo = port_config['readback_echo'] if 'readback_echo' in port_config else False
            min_motor_id = port_config['min_motor_id'] if 'min_motor_id' in port_config else 0
            max_motor_id = port_config['max_motor_id'] if 'max_motor_id' in port_config else 253
            update_rate = port_config['update_rate'] if 'update_rate' in port_config else 5
            error_level_temp = 75
            warn_level_temp = 70
            
            if 'diagnostics' in port_config:
                if 'error_level_temp' in port_config['diagnostics']:
                    error_level_temp = port_config['diagnostics']['error_level_temp']
                if 'warn_level_temp' in port_config['diagnostics']:
                    warn_level_temp = port_config['diagnostics']['warn_level_temp']
                    
            serial_proxy = SerialProxy(port_name,
                                       port_namespace,
                                       baud_rate,
                                       min_motor_id,
                                       max_motor_id,
                                       update_rate,
                                       self.diagnostics_rate,
                                       error_level_temp,
                                       warn_level_temp,
                                       readback_echo,
                                       node_namespace = self.get_namespace())
            serial_proxy.connect()
            
            # will create a set of services for each serial port under common manager namesapce
            # e.g. /dynamixel_manager/robot_arm_port/start_controller
            #      /dynamixel_manager/robot_head_port/start_controller
            # where 'dynamixel_manager' is manager's namespace
            #       'robot_arm_port' and 'robot_head_port' are human readable names for serial ports
            self.create_service(StartController, '%s/%s/start_controller' % (manager_namespace, port_namespace), self.start_controller)
            self.create_service(StopController, '%s/%s/stop_controller' % (manager_namespace, port_namespace), self.stop_controller)
            self.create_service(RestartController, '%s/%s/restart_controller' % (manager_namespace, port_namespace), self.restart_controller)
            
            self.serial_proxies[port_namespace] = serial_proxy
            
        # services for 'meta' controllers, e.g. joint trajectory controller
        # these controllers don't have their own serial port, instead they rely
        # on regular controllers for serial connection. The advantage of meta
        # controller is that it can pack commands for multiple motors on multiple
        # serial ports.
        # NOTE: all serial ports that meta controller needs should be managed by
        # the same controler manager.
        self.create_service(StartController, '%s/meta/start_controller' % manager_namespace, self.start_controller)
        self.create_service(StopController, '%s/meta/stop_controller' % manager_namespace, self.stop_controller)
        self.create_service(RestartController, '%s/meta/restart_controller' % manager_namespace, self.restart_controller)
        
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        if self.diagnostics_rate > 0: Thread(target=self.diagnostics_processor).start()

    def on_shutdown(self):
        for serial_proxy in self.serial_proxies.values():
            serial_proxy.disconnect()

    def diagnostics_processor(self):
        diag_msg = DiagnosticArray()
        
        rate = self.create_rate(self.diagnostics_rate)
        while rclpy.ok():
            diag_msg.status = []
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            
            for controller in self.controllers.values():
                try:
                    joint_state = controller.joint_state
                    temps = joint_state.motor_temps
                    max_temp = max(temps)
                    
                    status = DiagnosticStatus()
                    status.name = 'Joint Controller (%s)' % controller.joint_name
                    status.hardware_id = 'Robotis Dynamixel %s on port %s' % (str(joint_state.motor_ids), controller.port_namespace)
                    status.values.append(KeyValue('Goal', str(joint_state.goal_pos)))
                    status.values.append(KeyValue('Position', str(joint_state.current_pos)))
                    status.values.append(KeyValue('Error', str(joint_state.error)))
                    status.values.append(KeyValue('Velocity', str(joint_state.velocity)))
                    status.values.append(KeyValue('Load', str(joint_state.load)))
                    status.values.append(KeyValue('Moving', str(joint_state.is_moving)))
                    status.values.append(KeyValue('Temperature', str(max_temp)))
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'
                        
                    diag_msg.status.append(status)
                except:
                    pass
                    
            self.diagnostics_pub.publish(diag_msg)
            rate.sleep()

    def check_deps(self):
        controllers_still_waiting = []
        
        for i,(controller_name,deps,kls) in enumerate(self.waiting_meta_controllers):
            if not set(deps).issubset(self.controllers.keys()):
                controllers_still_waiting.append(self.waiting_meta_controllers[i])
                self.get_logger().warning('[%s] not all dependencies started, still waiting for %s...' % (controller_name, str(list(set(deps).difference(self.controllers.keys())))))
            else:
                dependencies = [self.controllers[dep_name] for dep_name in deps]
                controller = kls(controller_name, dependencies)
                
                if controller.initialize():
                    controller.start()
                    self.controllers[controller_name] = controller
                    rclpy.get_global_executor().add_node(controller)
                    
        self.waiting_meta_controllers = controllers_still_waiting[:]

    def start_controller(self, req, res):
        port_name = req.port_name
        package_path = req.package_path
        module_name = req.module_name
        class_name = req.class_name
        controller_name = req.controller_name
        parameters = [Parameter.from_parameter_msg(p) for p in req.parameters]
        
        self.start_controller_lock.acquire()
        
        if controller_name in self.controllers:
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Controller [%s] already started. If you want to restart it, call restart.' % controller_name
            return res
            
        try:
            if module_name not in sys.modules:
                # import if module not previously imported
                package_module = __import__(package_path, globals(), locals(), [module_name])
            else:
                # reload module if previously imported
                package_module = reload(sys.modules[package_path])
            controller_module = getattr(package_module, module_name)
        except ImportError as ie:
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Cannot find controller module. Unable to start controller %s\n%s' % (module_name, str(ie))
            return res
        except SyntaxError as se:
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Syntax error in controller module. Unable to start controller %s\n%s' % (module_name, str(se))
            return res
        except Exception as e:
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Unknown error has occured. Unable to start controller %s\n%s' % (module_name, str(e))
            return res
        
        kls = getattr(controller_module, class_name)
        
        if port_name == 'meta':
            self.waiting_meta_controllers.append((controller_name,req.dependencies,kls))
            self.check_deps()
            self.start_controller_lock.release()
            res.success = True
            res.reason = ''
            return res
            
        if port_name != 'meta' and (port_name not in self.serial_proxies):
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Specified port [%s] not found, available ports are %s. Unable to start controller %s' % (port_name, str(self.serial_proxies.keys()), controller_name)
            return res
            
        controller = kls(self.serial_proxies[port_name].dxl_io, controller_name, port_name,
                         node_namespace = self.get_namespace(),
                         parameters = parameters)
        
        if controller.initialize():
            controller.start()
            self.controllers[controller_name] = controller
            rclpy.get_global_executor().add_node(controller)
            
            self.check_deps()
            self.start_controller_lock.release()
            
            res.success = True
            res.reason = 'Controller %s successfully started.' % controller_name
            return res
        else:
            self.start_controller_lock.release()
            res.success = False
            res.reason = 'Initialization failed. Unable to start controller %s' % controller_name
            return res

    def stop_controller(self, req, res):
        controller_name = req.controller_name
        self.stop_controller_lock.acquire()
        
        if controller_name in self.controllers:
            self.controllers[controller_name].stop()
            rclpy.get_global_executor().remove_node(controllers[controller_name])
            del self.controllers[controller_name]
            self.stop_controller_lock.release()
            res.success = True
            res.reason = 'controller %s successfully stopped.' % controller_name
            return res
        else:
            self.self.stop_controller_lock.release()
            res.success = False
            res.reason = 'controller %s was not running.' % controller_name
            return res

    def restart_controller(self, req, res):
        response1 = self.stop_controller(StopController(req.controller_name))
        response2 = self.start_controller(req)
        res.success = response1.success and response2.success
        res.reason = '%s\n%s' % (response1.reason, response2.reason)
        return res

def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = ControllerManager()
        #rclpy.spin(manager)
        executor = rclpy.get_global_executor()
        executor.add_node(manager)
        for node in manager.serial_proxies.values():
            executor.add_node(node)
        executor.spin()
    finally:
        if manager is not None:
            manager.on_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

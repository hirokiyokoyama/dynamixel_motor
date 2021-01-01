#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
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


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import os
from optparse import OptionParser

import rclpy
from rclpy.node import Node

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController


parser = OptionParser()

def manage_controller(node, controller_name, port_namespace, controller_type, command, deps, start, stop, restart, set_parameters):
    try:
        package_path = node.get_parameter(controller_name + '.controller.package')
        module_name = node.get_parameter(controller_name + '.controller.module')
        class_name = node.get_parameter(controller_name + '.controller.type')
    except rclpy.exceptions.ParameterNotDeclaredException as e:
        node.get_logger().error('[%s] configuration error: could not find controller parameters on parameter server' % controller_name)
        raise
    except Exception as e:
        node.get_logger().error('[%s]: %s' % (controller_name, e))
        raise
        
    if command.lower() == 'start':
        try:
            req = StartController.Request()
            req.port_name = port_namespace
            req.package_path = package_path
            req.module_name = module_name
            req.class_name = class_name
            req.controller_name = controller_name
            req.dependencies = deps

            for param in node.get_parameters_by_prefix(controller_name):
                if not param.name.startswith(controller_name + '.controller.'):
                    req.parameters.append(param.to_parameter_msg())
            node.get_logger().info('[%s] passing %d parameters' % (controller_name, len(req.parameters)))
            
            response = start.call(req)
            if response.success: node.get_logger().info(response.reason)
            else: node.get_logger().error(response.reason)
        except Exception as e:
            node.get_logger().error('Service call failed: %s' % e)
    elif command.lower() == 'stop':
        try:
            req = StopController.Request()
            req.controller_name = controller_name
            response = stop.call(req)
            if response.success: node.get_logger().info(response.reason)
            else: node.get_logger().error(response.reason)
        except Exception as e:
            node.get_logger().error('Service call failed: %s' % e)
    elif command.lower() == 'restart':
        try:
            req = RestartController.Request()
            req.port_name = port_namespace
            req.package_path = package_path
            req.module_name = module_name
            req.class_name = class_name
            req.controller_name = controller_name
            req.dependencies = deps
            
            for param in node.get_parameters_by_prefix(controller_name):
                if not param.name.startswith(controller_name + '.controller.'):
                    req.parameters.append(param.to_parameter_msg())
            node.get_logger().info('[%s] passing %d parameters' % (controller_name, len(req.parameters)))

            response = restart.call(req)
            if response.success: node.get_logger().info(response.reason)
            else: node.get_logger().error(response.reason)
        except Exception as e:
            node.get_logger().error('Service call failed: %s' % e)
    else:
        node.get_logger().error('Invalid command.')
        parser.print_help()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = rclpy.create_node('controller_spawner')
        
        parser.add_option('-m', '--manager', metavar='MANAGER',
                          help='specified serial port is managed by MANAGER')
        parser.add_option('-p', '--port', metavar='PORT',
                          help='motors of specified controllers are connected to PORT')
        parser.add_option('-t', '--type', metavar='TYPE', default='simple', choices=('simple','meta'),
                          help='type of controller to be loaded (simple|meta) [default: %default]')
        parser.add_option('-c', '--command', metavar='COMMAND', default='start', choices=('start','stop','restart'),
                          help='command to perform on specified controllers: start, stop or restart [default: %default]')
                          
        (options, args) = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])
        
        if len(args) < 1:
            parser.error('specify at least one controller name')
            
        manager_namespace = options.manager
        port_namespace = options.port
        controller_type = options.type
        command = options.command
        joint_controllers = args
        
        if controller_type == 'meta': port_namespace = 'meta'
        
        start_service_name = '%s/%s/start_controller' % (manager_namespace, port_namespace)
        stop_service_name = '%s/%s/stop_controller' % (manager_namespace, port_namespace)
        restart_service_name = '%s/%s/restart_controller' % (manager_namespace, port_namespace)

        parent_namespace = 'global' if node.get_namespace() == '/' else node.get_namespace()
        node.get_logger().info('%s controller_spawner: waiting for controller_manager %s to startup in %s namespace...' % (port_namespace, manager_namespace, parent_namespace))
        
        
        start_controller = node.create_client(StartController, start_service_name)
        stop_controller = node.create_client(StopController, stop_service_name)
        restart_controller = node.create_client(RestartController, restart_service_name)
        start_controller.wait_for_service()
        stop_controller.wait_for_service()
        restart_controller.wait_for_service()
        
        node.get_logger().info('%s controller_spawner: All services are up, spawning controllers...' % port_namespace)
        
        if controller_type == 'simple':
            for controller_name in joint_controllers:
                manage_controller(node, controller_name, port_namespace, controller_type, command, [], start_controller, stop_controller, restart_controller)
        elif controller_type == 'meta':
            controller_name = joint_controllers[0]
            dependencies = joint_controllers[1:]
            manage_controller(node, controller_name, port_namespace, controller_type, command, dependencies, start_controller, stop_controller, restart_controller)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

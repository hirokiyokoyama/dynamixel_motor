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


import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

from dynamixel_driver.dynamixel_const import *

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetComplianceSlope
from dynamixel_controllers.srv import SetComplianceMargin
from dynamixel_controllers.srv import SetCompliancePunch
from dynamixel_controllers.srv import SetTorqueLimit

from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import JointState

class JointController(Node):
    def __init__(self, dxl_io, controller_namespace, port_namespace,
                 node_namespace = None,
                 parameters = None):
        super().__init__(
            controller_namespace,
            namespace = node_namespace,
            parameter_overrides = parameters)
        
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.joint_name = self.declare_parameter('joint_name').value
        self.joint_speed = self.declare_parameter('joint_speed', 1.0).value
        self.compliance_slope = self.declare_parameter('joint_compliance_slope').value
        self.compliance_margin = self.declare_parameter('joint_compliance_margin').value
        self.compliance_punch = self.declare_parameter('joint_compliance_punch').value
        self.torque_limit = self.declare_parameter('joint_torque_limit').value
        
        self.__ensure_limits()
        
        self.speed_service = self.create_service(SetSpeed, self.controller_namespace + '/set_speed',  self.process_set_speed)
        self.torque_service = self.create_service(TorqueEnable, self.controller_namespace + '/torque_enable', self.process_torque_enable)
        self.compliance_slope_service = self.create_service(SetComplianceSlope, self.controller_namespace + '/set_compliance_slope', self.process_set_compliance_slope)
        self.compliance_marigin_service = self.create_service(SetComplianceMargin, self.controller_namespace + '/set_compliance_margin', self.process_set_compliance_margin)
        self.compliance_punch_service = self.create_service(SetCompliancePunch, self.controller_namespace + '/set_compliance_punch', self.process_set_compliance_punch)
        self.torque_limit_service = self.create_service(SetTorqueLimit, self.controller_namespace + '/set_torque_limit', self.process_set_torque_limit)

        self.dynamixel_parameter_client = self.create_client(GetParameters, '/{}/get_parameters'.format(port_namespace))
        self.dynamixel_parameter_client.wait_for_service()
        #rclpy.get_global_executor().add_node(self)

    def __ensure_limits(self):
        if self.compliance_slope is not None:
            if self.compliance_slope < DXL_MIN_COMPLIANCE_SLOPE: self.compliance_slope = DXL_MIN_COMPLIANCE_SLOPE
            elif self.compliance_slope > DXL_MAX_COMPLIANCE_SLOPE: self.compliance_slope = DXL_MAX_COMPLIANCE_SLOPE
            else: self.compliance_slope = int(self.compliance_slope)
            
        if self.compliance_margin is not None:
            if self.compliance_margin < DXL_MIN_COMPLIANCE_MARGIN: self.compliance_margin = DXL_MIN_COMPLIANCE_MARGIN
            elif self.compliance_margin > DXL_MAX_COMPLIANCE_MARGIN: self.compliance_margin = DXL_MAX_COMPLIANCE_MARGIN
            else: self.compliance_margin = int(self.compliance_margin)
            
        if self.compliance_punch is not None:
            if self.compliance_punch < DXL_MIN_PUNCH: self.compliance_punch = DXL_MIN_PUNCH
            elif self.compliance_punch > DXL_MAX_PUNCH: self.compliance_punch = DXL_MAX_PUNCH
            else: self.compliance_punch = int(self.compliance_punch)
            
        if self.torque_limit is not None:
            if self.torque_limit < 0: self.torque_limit = 0.0
            elif self.torque_limit > 1: self.torque_limit = 1.0

    def initialize(self):
        raise NotImplementedError

    def start(self):
        self.running = True
        self.joint_state_pub = self.create_publisher(JointState, self.controller_namespace + '/state', 1)
        self.command_sub = self.create_subscription(Float64, self.controller_namespace + '/command', self.process_command, 1)
        self.motor_states_sub = self.create_subscription(MotorStateList, 'motor_states/%s' % self.port_namespace, self.process_motor_states, 1)

    def stop(self):
        self.running = False
        self.joint_state_pub.destroy()
        self.motor_states_sub.destroy()
        self.command_sub.destroy()
        self.speed_service.destroy()
        self.torque_service.destroy()
        self.compliance_slope_service.destroy()

    def set_torque_enable(self, torque_enable):
        raise NotImplementedError

    def set_speed(self, speed):
        raise NotImplementedError

    def set_compliance_slope(self, slope):
        raise NotImplementedError

    def set_compliance_margin(self, margin):
        raise NotImplementedError

    def set_compliance_punch(self, punch):
        raise NotImplementedError

    def set_torque_limit(self, max_torque):
        raise NotImplementedError

    def process_set_speed(self, req, res):
        self.set_speed(req.speed)
        return res # success

    def process_torque_enable(self, req, res):
        self.set_torque_enable(req.torque_enable)
        return res

    def process_set_compliance_slope(self, req, res):
        self.set_compliance_slope(req.slope)
        return res

    def process_set_compliance_margin(self, req, res):
        self.set_compliance_margin(req.margin)
        return res

    def process_set_compliance_punch(self, req, res):
        self.set_compliance_punch(req.punch)
        return res

    def process_set_torque_limit(self, req, res):
        self.set_torque_limit(req.torque_limit)
        return res

    def process_motor_states(self, state_list):
        raise NotImplementedError

    def process_command(self, msg):
        raise NotImplementedError

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

    def get_dynamixel_parameter(self, name, default_value=None):
        try:
            req = GetParameters.Request()
            req.names.append(name)
            self.get_logger().info('Getting dynamixel parameter %s' % name)
            future = self.dynamixel_parameter_client.call_async(req)
            #rclpy.get_global_executor().spin_until_future_complete(future)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res is not None:
                value = None
                pvalue = res.values[0]
                if pvalue.type == ParameterType.PARAMETER_BOOL:
                    value = pvalue.bool_value
                elif pvalue.type == ParameterType.PARAMETER_INTEGER:
                    value = pvalue.integer_value
                elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
                    value = pvalue.double_value
                elif pvalue.type == ParameterType.PARAMETER_STRING:
                    value = pvalue.string_value
                elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
                    value = pvalue.byte_array_value
                elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
                    value = pvalue.bool_array_value
                elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    value = pvalue.integer_array_value
                elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    value = pvalue.double_array_value
                elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                    value = pvalue.string_array_value
                elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                    value = None
                if value is not None:
                    return value
            if default_value is not None:
                return default_value
            raise RuntimeError()
        except Exception as e:
            self.get_logger().error('Could not get dynamixel parameter %s' % name)
            raise

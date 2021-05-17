#! /usr/bin/env python3

# Copyright (c) 2020-2021, Arm Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import numpy as np
from rclpy.node import Node


def getParameter(node: Node, param_name: str):
    param = node.get_parameter(param_name)
    if param is None:
        node.get_logger().error("Parameter %s not found." % param_name)
        sys.exit(-1)
    return param.value


def info(node: Node, text: str):
    node.get_logger().info(text)


def warning(node: Node, text: str):
    node.get_logger().warning(text)


def error(node: Node, text: str):
    node.get_logger().error(text)


def euler_from_quaternion(w, x, y, z):
    ysqr = y * y

    sinr_cosp = 2.0 * ((w * x) + (y * z))
    cosr_cosp = 1.0 - (2.0 * ((x * x) + ysqr))
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * ((w * y) - (z * x))

    sinp = np.clip(sinp, a_min=-1.0, a_max=1.0)
    pitch = np.degrees(np.arcsin(sinp))

    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * (ysqr + (z * z)))
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from curses.ascii import isupper
from typing import List
from enum import Enum

class ProtonROS2MessageConfig:
    # Top level keys
    PACKAGE = "package"
    MESSAGES = "messages"

    class Message:
        # Message keys
        NAME = "name"
        PATH = "path"
        MAPPING = "mapping"
        STAMP = "stamp"

        class Mapping:
            ROS2_PATH = "ros2.path"
            ROS2_INDEX = "ros2.index"
            ROS2_LENGTH = "ros2.length"
            ROS2_SUBPATH = "ros2.subpath"
            PROTON_SIGNAL = "proton.signal"
            PROTON_INDEX = "proton.index"
            TYPE = "type"


            class MappingType(Enum):
                SCALAR = 0
                PROTON_INDEXED = 1
                ROS_INDEXED = 2
                BOTH_INDEXED = 3
                FIXED_LIST = 4
                DYNAMIC_LIST = 5
                FIXED_SUBMESSAGE = 6
                DYNAMIC_SUBMESSAGE = 7


            def __init__(self, config: dict):
                self.config = config

                # Required fields
                self.ros_path = config[self.ROS2_PATH]
                self.proton_signal = config[self.PROTON_SIGNAL]
                self.data_type = config[self.TYPE]
                self.mapping_type = 0

                # Optional fields
                try:
                    self.proton_index = config[self.PROTON_INDEX]
                except KeyError:
                    self.proton_index = None

                try:
                    self.ros_index = config[self.ROS2_INDEX]
                except KeyError:
                    self.ros_index = None

                try:
                    self.ros_length = config[self.ROS2_LENGTH]
                except KeyError:
                    self.ros_length = None

                try:
                    self.ros_subpath = config[self.ROS2_SUBPATH]
                except KeyError:
                    self.ros_subpath = None

                # ros_msg.field = proton.signal
                if (self.proton_index is None and
                    self.ros_index is None and
                    self.ros_length is None and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.SCALAR
                # ros_msg.field = proton.signal[proton_index]
                elif(self.proton_index is not None and
                    self.ros_index is None and
                    self.ros_length is None and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.PROTON_INDEXED
                # ros_msg.field[ros_index] = proton.signal
                elif(self.proton_index is None and
                    self.ros_index is not None and
                    self.ros_length is None and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.ROS_INDEXED
                # ros_msg.field[ros_index] = proton.signal[proton_index]
                elif(self.proton_index is not None and
                    self.ros_index is not None and
                    self.ros_length is None and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.BOTH_INDEXED
                # ros_msg.field[n] = proton.signal[n]
                elif(self.proton_index is None and
                    self.ros_index is None and
                    self.ros_length > 0 and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.FIXED_LIST
                # ros_msg.field[] = proton.signal[]
                elif(self.proton_index is None and
                    self.ros_index is None and
                    self.ros_length == 0 and
                    self.ros_subpath is None):
                    self.mapping_type = self.MappingType.DYNAMIC_LIST
                # ros_msg.field[n].subfield = proton.signal[n]
                elif(self.proton_index is None and
                    self.ros_index is None and
                    self.ros_length > 0 and
                    self.ros_subpath is not None):
                    self.mapping_type = self.MappingType.FIXED_SUBMESSAGE
                # ros_msg.field[].subfield = proton.signal[]
                elif(self.proton_index is None and
                    self.ros_index is None and
                    self.ros_length == 0 and
                    self.ros_subpath is not None):
                    self.mapping_type = self.MappingType.DYNAMIC_SUBMESSAGE
                else:
                    raise KeyError(f"Invalid mapping configuration: {config}")


        def __init__(self, package: str, config: dict):
            self.package = package
            self.path = config[self.PATH]
            self.config = config
            self.name = config[self.NAME]
            self.snakecase_name = ProtonROS2MessageConfig.camelcase_to_snakecase(self.name)
            self.full_name = f'{package}/{self.path}/{self.name}'
            self.ros_type = f'{package}::{self.path}::{self.name}'
            self.hpp_header = f'{package}/{self.path}/{self.snakecase_name}.hpp'

            try:
                self.stamp = config[self.STAMP]
            except KeyError:
                self.stamp = None

            self.mappings: List[ProtonROS2MessageConfig.Message.Mapping] = []
            for m in config[self.MAPPING]:
                self.mappings.append(self.Mapping(m))

    def camelcase_to_snakecase(string: str):
        scase: str = ""
        i = 0
        if string.isupper():
            return string.lower()

        for c in string:
            if c.isupper():
                if i == 0:
                    scase += c.lower()
                else:
                    scase += "_" + c.lower()
            else:
                scase += c
            i += 1
        return scase

    def __init__(self, config: dict):
        self.config = config
        self.package = config[self.PACKAGE]
        self.messages: List[ProtonROS2MessageConfig.Message] = []
        for m in config[self.MESSAGES]:
            self.messages.append(self.Message(self.package, m))

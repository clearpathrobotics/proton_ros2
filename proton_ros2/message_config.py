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

from typing import List
from enum import Enum

class ProtonROS2Config:
    # Top level keys
    PACKAGE = "package"
    MESSAGES = "messages"
    SERVICES = "services"

    class Mapping:
        ROS2_PATH = "ros2.path"
        ROS2_INDEX = "ros2.index"
        ROS2_LENGTH = "ros2.length"
        ROS2_SUBPATH = "ros2.subpath"
        PROTON_SIGNAL = "proton.signal"
        PROTON_INDEX = "proton.index"
        PROTON_SUBINDEX = "proton.subindex"
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
            FIXED_SUBINDEX = 8
            DYNAMIC_SUBINDEX = 9


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
                self.proton_subindex = config[self.PROTON_SUBINDEX]
            except KeyError:
                self.proton_subindex = None

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
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length is None and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.SCALAR
            # ros_msg.field = proton.signal[proton_index]
            elif(self.proton_index is not None and
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length is None and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.PROTON_INDEXED
            # ros_msg.field[ros_index] = proton.signal
            elif(self.proton_index is None and
                self.proton_subindex is None and
                self.ros_index is not None and
                self.ros_length is None and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.ROS_INDEXED
            # ros_msg.field[ros_index] = proton.signal[proton_index]
            elif(self.proton_index is not None and
                self.proton_subindex is None and
                self.ros_index is not None and
                self.ros_length is None and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.BOTH_INDEXED
            # ros_msg.field[n] = proton.signal[n]
            elif(self.proton_index is None and
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length > 0 and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.FIXED_LIST
            # ros_msg.field[] = proton.signal[]
            elif(self.proton_index is None and
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length == 0 and
                self.ros_subpath is None):
                self.mapping_type = self.MappingType.DYNAMIC_LIST
            # ros_msg.field[n].subfield = proton.signal[n]
            elif(self.proton_index is None and
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length > 0 and
                self.ros_subpath is not None):
                self.mapping_type = self.MappingType.FIXED_SUBMESSAGE
            # ros_msg.field[].subfield = proton.signal[]
            elif(self.proton_index is None and
                self.proton_subindex is None and
                self.ros_index is None and
                self.ros_length == 0 and
                self.ros_subpath is not None):
                self.mapping_type = self.MappingType.DYNAMIC_SUBMESSAGE
            # ros_msg.field[n].subfield = proton.signal[n][proton_subindex]
            elif(self.proton_index is None and
                self.proton_subindex is not None and
                self.ros_index is None and
                self.ros_length > 0 and
                self.ros_subpath is not None and
                self.data_type == "list_bytes"):
                self.mapping_type = self.MappingType.FIXED_SUBINDEX
            # ros_msg.field[].subfield = proton.signal[n][proton_subindex]
            elif(self.proton_index is None and
                self.proton_subindex is not None and
                self.ros_index is None and
                self.ros_length == 0 and
                self.ros_subpath is not None and
                self.data_type == "list_bytes"):
                self.mapping_type = self.MappingType.DYNAMIC_SUBINDEX
            else:
                raise KeyError(f"Invalid mapping configuration: {config}")

    class Message:
        # Message keys
        NAME = "name"
        PATH = "path"
        MAPPING = "mapping"
        STAMP = "stamp"

        def __init__(self, package: str, config: dict):
            self.package = package
            self.config = config
            self.name = config[self.NAME]
            self.path = config[self.PATH]
            self.snakecase_name = ProtonROS2Config.camelcase_to_snakecase(self.name)
            self.full_name = f'{package}/{self.path}/{self.name}'
            self.ros_type = f'{package}::{self.path}::{self.name}'
            self.hpp_header = f'{package}/{self.path}/{self.snakecase_name}.hpp'

            try:
                self.stamp = config[self.STAMP]
            except KeyError:
                self.stamp = None

            self.mappings: List[ProtonROS2Config.Mapping] = []
            for m in config[self.MAPPING]:
                self.mappings.append(ProtonROS2Config.Mapping(m))

    class Service:
        # Message keys
        NAME = "name"
        PATH = "path"
        MAPPING = "mapping"
        REQUEST = "request"
        RESPONSE = "response"

        def __init__(self, package: str, config: dict):
            self.package = package
            self.config = config
            self.name = config[self.NAME]
            self.path = config[self.PATH]
            self.snakecase_name = ProtonROS2Config.camelcase_to_snakecase(self.name)
            self.full_name = f'{package}/{self.path}/{self.name}'
            self.ros_type = f'{package}::{self.path}::{self.name}'
            self.hpp_header = f'{package}/{self.path}/{self.snakecase_name}.hpp'


            self.request_mappings: List[ProtonROS2Config.Mapping] = []
            self.response_mappings: List[ProtonROS2Config.Mapping] = []
            try:
                for m in config[self.MAPPING][self.REQUEST]:
                    self.request_mappings.append(ProtonROS2Config.Mapping(m))
            except KeyError:
                pass

            try:
                for m in config[self.MAPPING][self.RESPONSE]:
                    self.response_mappings.append(ProtonROS2Config.Mapping(m))
            except KeyError:
                pass


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

        self.messages: List[ProtonROS2Config.Message] = []
        self.services: List[ProtonROS2Config.Service] = []
        try:
            for m in config[self.MESSAGES]:
                self.messages.append(self.Message(self.package, m))
        except KeyError:
            pass

        try:
            for s in config[self.SERVICES]:
                print(s)
                self.services.append(self.Service(self.package, s))
        except KeyError:
            pass

# Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
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
#
# @author Roni Kreinin (roni.kreinin@rockwellautomation.com)

from typing import List
from enum import Enum
import re

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
        SKIP = "skip"
        MAPPING = "mapping"
        STAMP = "stamp"

        def __init__(self, package: str, config: dict):
            self.package = package
            self.config = config
            self.name = config[self.NAME]
            self.path = config[self.PATH]
            self.snakecase_name = ProtonROS2Config.camel_to_snake(self.name)
            self.full_name = f'{package}/{self.path}/{self.name}'
            self.ros_type = f'{package}::{self.path}::{self.name}'
            self.hpp_header = f'{package}/{self.path}/{self.snakecase_name}.hpp'

            try:
                self.skip = config[self.SKIP]
            except KeyError:
                self.skip = False

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
        SKIP = "skip"
        MAPPING = "mapping"
        REQUEST = "request"
        RESPONSE = "response"

        def __init__(self, package: str, config: dict):
            self.package = package
            self.config = config
            self.name = config[self.NAME]
            self.path = config[self.PATH]
            self.snakecase_name = ProtonROS2Config.camel_to_snake(self.name)
            self.full_name = f'{package}/{self.path}/{self.name}'
            self.ros_type = f'{package}::{self.path}::{self.name}'
            self.hpp_header = f'{package}/{self.path}/{self.snakecase_name}.hpp'


            self.request_mappings: List[ProtonROS2Config.Mapping] = []
            self.response_mappings: List[ProtonROS2Config.Mapping] = []

            try:
                self.skip = config[self.SKIP]
            except KeyError:
                self.skip = False

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

    def camel_to_snake(name):
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

    def __init__(self, config: dict):
        self.config = config
        self.package = config[self.PACKAGE]

        self.messages: List[ProtonROS2Config.Message] = []
        self.services: List[ProtonROS2Config.Service] = []
        for m in config[self.MESSAGES]:
            try:
                self.messages.append(self.Message(self.package, m))
            except KeyError:
                continue

        for s in config[self.SERVICES]:
            try:
                self.services.append(self.Service(self.package, s))
            except KeyError:
                continue

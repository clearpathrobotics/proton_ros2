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
            ROS_PATH = "ros_path"
            SIGNAL = "signal"
            TYPE = "type"
            LENGTH = "length"
            INDEX = "index"

            def __init__(self, config: dict):
                self.config = config
                self.ros_path = config[self.ROS_PATH]
                self.signal = config[self.SIGNAL]
                self.type = config[self.TYPE]

                try:
                    self.length = config[self.LENGTH]
                except KeyError:
                    self.length = None

                try:
                    self.index = config[self.INDEX]
                except KeyError:
                    self.index = None

        def __init__(self, package: str, config: dict):
            self.package = package
            self.path = config[self.PATH]
            self.config = config
            self.name = config[self.NAME]
            self.full_name = f'{package}/{self.path}/{self.name.lower()}'
            self.ros_type = f'{package}::{self.path}::{self.name}'

            try:
                self.stamp = config[self.STAMP]
            except KeyError:
                self.stamp = None

            self.mappings: List[ProtonROS2MessageConfig.Message.Mapping] = []
            for m in config[self.MAPPING]:
                self.mappings.append(self.Mapping(m))

    def __init__(self, config: dict):
        self.config = config
        self.package = config[self.PACKAGE]
        self.messages: List[ProtonROS2MessageConfig.Message] = []
        for m in config[self.MESSAGES]:
            self.messages.append(self.Message(self.package, m))

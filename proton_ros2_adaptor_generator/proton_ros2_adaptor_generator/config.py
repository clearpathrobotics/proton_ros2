
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

"""
Configuration dataclasses for proton_ros2_adaptor_generator.

These dataclasses represent the YAML configuration format and provide
methods for parsing and validation.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import yaml


class MappingType(Enum):
    """Classification of signal-to-field mapping types."""

    SCALAR = auto()  # Direct field: msg.data -> signal
    ROS_INDEXED = auto()  # Array with fixed index: msg.temps[0] -> signal


@dataclass
class Mapping:
    """A single field-to-signal mapping."""

    ros_path: str  # Dot-separated path in ROS message (e.g., "header.frame_id")
    signal_name: str  # Proton signal name
    data_type: str  # Proton type: double, float, int32, int64, uint32, uint64, bool, string, bytes
    ros_index: Optional[int] = None  # Array index for ROS_INDEXED mappings

    # Resolved at generation time
    signal_id: Optional[str] = None  # Signal ID constant name (e.g., "SIGNAL_ID_TEMPERATURE")

    @property
    def mapping_type(self) -> MappingType:
        """Determine mapping type from configuration."""
        if self.ros_index is not None:
            return MappingType.ROS_INDEXED
        return MappingType.SCALAR

    @classmethod
    def from_dict(cls, d: dict) -> "Mapping":
        """Parse a mapping from YAML dict format."""
        return cls(
            ros_path=d["ros2.path"],
            signal_name=d["proton.signal"],
            data_type=d["type"],
            ros_index=d.get("ros2.index"),
        )


@dataclass
class MessageBinding:
    """Configuration for a single message adaptor binding."""

    name: str  # Unique binding name (e.g., "BoardTemps")
    ros2_type: str  # Fully qualified ROS type (e.g., "clearpath_platform_msgs/msg/Temperature")
    mappings: list[Mapping] = field(default_factory=list)
    stamp_path: Optional[str] = None  # Path to timestamp field for injection (e.g., "header.stamp")

    @property
    def adaptor_class(self) -> str:
        """Generate C++ class name for this adaptor."""
        return f"{self.name}Adaptor"

    @property
    def source_file(self) -> str:
        """Generate source filename for this adaptor."""
        # Convert CamelCase to snake_case
        import re

        name = re.sub(r"(?<!^)(?=[A-Z])", "_", self.name).lower()
        return f"{name}_adaptor.cpp"

    @property
    def ros_cpp_type(self) -> str:
        """Convert ROS type to C++ type (e.g., 'geometry_msgs/msg/Twist' -> 'geometry_msgs::msg::Twist')."""
        return self.ros2_type.replace("/", "::")

    @property
    def hpp_include(self) -> str:
        """Generate C++ include path for ROS message header."""
        return self.ros2_type.lower().replace("/msg/", "/msg/").replace("/srv/", "/srv/") + ".hpp"

    @property
    def ros_package(self) -> str:
        """Extract ROS package name from type."""
        return self.ros2_type.split("/")[0]

    @classmethod
    def from_dict(cls, d: dict) -> "MessageBinding":
        """Parse a message binding from YAML dict format."""
        mappings = [Mapping.from_dict(m) for m in d.get("mapping", [])]
        return cls(
            name=d["name"],
            ros2_type=d["ros2_type"],
            mappings=mappings,
            stamp_path=d.get("stamp"),
        )


@dataclass
class AdaptorConfig:
    """Complete configuration for an adaptor package."""

    messages: list[MessageBinding] = field(default_factory=list)

    @property
    def ros_msg_dependencies(self) -> set[str]:
        """Collect all ROS message package dependencies."""
        deps = set()
        for msg in self.messages:
            deps.add(msg.ros_package)
        return deps

    @classmethod
    def from_yaml(cls, path: Path) -> "AdaptorConfig":
        """Load configuration from a YAML file."""
        with open(path) as f:
            data = yaml.safe_load(f)

        messages = [MessageBinding.from_dict(m) for m in data.get("messages", [])]

        return cls(messages=messages)

    def validate(self) -> list[str]:
        """Validate configuration and return list of errors."""
        errors = []

        # Check for duplicate binding names
        names = [m.name for m in self.messages]
        seen = set()
        for name in names:
            if name in seen:
                errors.append(f"Duplicate binding name: {name}")
            seen.add(name)

        # Validate data types
        valid_types = {"double", "float", "int32", "int64", "uint32", "uint64", "bool", "string", "bytes"}
        for msg in self.messages:
            for mapping in msg.mappings:
                if mapping.data_type not in valid_types:
                    errors.append(f"Invalid data type '{mapping.data_type}' in binding '{msg.name}'")

        return errors


@dataclass
class PackageConfig:
    """Configuration for the generated package metadata."""

    package_name: str
    project_name: str
    version: str = "0.0.1"
    maintainer_name: str = "Unknown"
    maintainer_email: str = "unknown@example.com"
    license: str = "Apache-2.0"

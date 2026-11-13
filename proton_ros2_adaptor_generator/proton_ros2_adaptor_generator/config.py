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

import re
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

import yaml

# One dotted path component: a field name followed by any number of [index] suffixes
_PATH_COMPONENT = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)((?:\[\d+\])*)$")


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
    ros_index: int | None = None  # Array index for ROS_INDEXED mappings
    proton_index: int | None = None  # Byte of a bytes signal that holds this field

    # Resolved at generation time
    signal_id: int | None = None  # Signal ID integer
    signal_capacity: int | None = (
        None  # Signal capacity for repeated types (bytes, string)
    )

    @property
    def mapping_type(self) -> MappingType:
        """Determine mapping type from configuration."""
        if self.ros_index is not None or "[" in self.ros_path:
            return MappingType.ROS_INDEXED
        return MappingType.SCALAR

    def __post_init__(self) -> None:
        """Turn a single trailing "[n]" on the path into ros_index, as ros2.index would."""
        match = re.fullmatch(r"([^\[\]]+)\[(\d+)\]", self.ros_path)
        if match and self.ros_index is None:
            self.ros_path = match.group(1)
            self.ros_index = int(match.group(2))

    @property
    def path_error(self) -> str | None:
        """Describe what is wrong with ros_path, or None if it is usable."""
        for part in self.ros_path.split("."):
            if _PATH_COMPONENT.match(part) is None:
                return f'invalid component "{part}" in ros2.path "{self.ros_path}"'
        if "[" in self.ros_path and self.ros_index is not None:
            return f'use either ros2.index or [n] in ros2.path "{self.ros_path}"'
        return None

    @property
    def components(self) -> list[tuple[str, list[int]]]:
        """Split the path into components with array indexes; ros_index goes on the last."""
        parts = []
        for part in self.ros_path.split("."):
            match = _PATH_COMPONENT.match(part)
            parts.append(
                (match.group(1), [int(i) for i in re.findall(r"\d+", match.group(2))])
            )
        if self.ros_index is not None:
            parts[-1][1].append(self.ros_index)
        return parts

    @property
    def nested_access(self) -> bool:
        """Return False only when the one array index is a single index on the last component."""
        counts = [len(indexes) for _, indexes in self.components]
        return sum(counts) > 1 or (sum(counts) == 1 and counts[-1] != 1)

    def _expression(self, accessor: str) -> str:
        expression = "msg"
        for name, indexes in self.components:
            expression += f".{name}"
            for index in indexes:
                expression = (
                    f"proton_ros2_interfaces::{accessor}<{index}>({expression})"
                )
        return expression

    @property
    def write_expr(self) -> str:
        """Return the C++ reference to the ROS field for writing; dynamic arrays grow to fit."""
        return self._expression("indexed_element")

    @property
    def read_expr(self) -> str:
        """Return the C++ reference to the ROS field for reading; a missing element throws."""
        return self._expression("indexed_field")

    @classmethod
    def from_dict(cls, d: dict) -> "Mapping":
        """Parse a mapping from YAML dict format."""
        return cls(
            ros_path=d['ros2.path'],
            signal_name=d['proton.signal'],
            data_type=d['type'],
            ros_index=d.get('ros2.index'),
            proton_index=d.get('proton.index'),
        )


@dataclass
class ArrayBinding:
    """A variable-length ROS array backed by ordered signals and a count signal."""

    ros_path: str
    elements: list[Mapping]
    num_elements: Mapping

    @classmethod
    def from_dict(cls, data: dict) -> 'ArrayBinding':
        """Parse an array binding; signal types are resolved from the Proton schema."""
        path = data['ros2.path']
        return cls(
            ros_path=path,
            elements=[Mapping(path, name, '', ros_index=index)
                      for index, name in enumerate(data['elements'])],
            num_elements=Mapping(path, data['num_elements'], ''),
        )


@dataclass
class MessageBinding:
    """Configuration for a single message adaptor binding."""

    name: str  # Unique binding name (e.g., "BoardTemps")
    ros2_type: str  # Fully qualified ROS type (e.g., "clearpath_platform_msgs/msg/Temperature")
    mappings: list[Mapping] = field(default_factory=list)
    # Path to timestamp field for injection (e.g., "header.stamp")
    stamp_path: str | None = None
    # Path for specific include names (e.g., std_msgs/msg/color_rgba.hpp)
    hpp_path: str | None = None
    arrays: list[ArrayBinding] = field(default_factory=list)

    @property
    def all_mappings(self) -> list[Mapping]:
        """Collect scalar and array signal references for schema resolution."""
        return self.mappings + [mapping for array in self.arrays
                                for mapping in [array.num_elements, *array.elements]]

    @property
    def adaptor_class(self) -> str:
        """Generate C++ class name for this adaptor."""
        return f'{self.name}Adaptor'

    @property
    def source_file(self) -> str:
        """Generate source filename for this adaptor."""
        # Convert CamelCase to snake_case

        name = re.sub(r'(?<!^)(?=[A-Z])', '_', self.name).lower()
        return f'{name}_adaptor.cpp'

    @property
    def ros_cpp_type(self) -> str:
        """
        Convert ROS type to C++ type.

        (e.g., 'geometry_msgs/msg/Twist' -> 'geometry_msgs::msg::Twist').
        """
        return self.ros2_type.replace('/', '::')

    @property
    def hpp_include(self) -> str:
        """
        Generate C++ include path for ROS message header.

        (e.g., 'std_msgs/msg/Float32' -> 'std_msgs/msg/float32.hpp'
        or
        'std_msgs/msg/UInt32' -> 'std_msgs/msg/u_int32.hpp').

        Key is that the header path is the snake_case version of the PascalCase message name,
        but without an underscore preceding numbers.
        """
        if self.hpp_path is not None:
            return self.hpp_path
        hpp = re.sub(r'(?<!^)(?=[A-Z])', '_', self.ros2_type).replace('msg/_', 'msg/').lower()
        return hpp + '.hpp'

    @property
    def ros_package(self) -> str:
        """Extract ROS package name from type."""
        return self.ros2_type.split('/')[0]

    @classmethod
    def from_dict(cls, d: dict) -> 'MessageBinding':
        """Parse a message binding from YAML dict format."""
        mappings = [Mapping.from_dict(m) for m in d.get('mapping', [])]
        return cls(
            name=d['name'],
            ros2_type=d['ros2_type'],
            mappings=mappings,
            stamp_path=d.get('stamp'),
            hpp_path=d.get('hpp_path'),
            arrays=[ArrayBinding.from_dict(array) for array in d.get('arrays', [])],
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
    def from_yaml(cls, path: Path) -> 'AdaptorConfig':
        """Load configuration from a YAML file."""
        with open(path) as f:
            data = yaml.safe_load(f)

        messages = [MessageBinding.from_dict(m) for m in data.get('messages', [])]

        return cls(messages=messages)

    def resolve_signal_ids(self, proton_config_path: Path) -> list[str]:
        """
        Resolve signal IDs from proton config file.

        Looks up each mapping's signal_name in the proton config's signals
        stanza and sets the signal_id field to the integer ID.

        :param proton_config_path: Path to proton YAML config with signals stanza
        :return: List of error messages (empty if all resolved successfully)
        """
        # Load proton config
        with open(proton_config_path) as f:
            proton_data = yaml.safe_load(f)

        # Build signal name -> id lookup
        signal_lookup: dict[str, int] = {}
        for sig in proton_data.get('signals', []):
            name = sig.get('name')
            sig_id = sig.get('id')
            if name and sig_id is not None:
                # Handle hex strings (e.g., '0x1000') or integers
                if isinstance(sig_id, str):
                    sig_id = int(sig_id, 0)  # auto-detect base (handles 0x prefix)
                signal_lookup[name] = sig_id

        signal_types = {sig['name']: sig.get('type') for sig in proton_data.get('signals', [])}

        # Resolve each mapping's signal_id
        errors = []
        for msg in self.messages:
            for mapping in msg.all_mappings:
                if mapping.signal_name in signal_lookup:
                    mapping.signal_id = signal_lookup[mapping.signal_name]
                    if not mapping.data_type:
                        mapping.data_type = signal_types[mapping.signal_name]
                else:
                    errors.append(
                        f'Signal "{mapping.signal_name}" not found in proton config '
                        f'(binding "{msg.name}", field "{mapping.ros_path}")'
                    )
            for array in msg.arrays:
                if (array.num_elements.signal_id is not None and
                    array.num_elements.data_type not in {
                        'int32', 'int64', 'uint32', 'uint64'}):
                    errors.append(f'Array count signal "{array.num_elements.signal_name}" '
                                  f'must have an integer type (binding "{msg.name}")')
                for element in array.elements:
                    if element.signal_id is not None and element.data_type not in {
                            'double', 'float', 'int32', 'int64', 'uint32', 'uint64',
                            'bool', 'string', 'bytes'}:
                        errors.append(f'Invalid array element type "{element.data_type}" '
                                      f'(binding "{msg.name}", signal "{element.signal_name}")')

        return errors

    def resolve_signal_capacities(self, proton_config_path: Path) -> list[str]:
        """
        Resolve signal capacities for repeated types from proton config.

        Looks up each mapping's signal_name in the proton config and sets
        the signal_capacity field for string/bytes types.

        :param proton_config_path: Path to proton YAML config with signals stanza
        :return: List of error messages (empty if all resolved successfully)
        """
        # Load proton config
        with open(proton_config_path) as f:
            proton_data = yaml.safe_load(f)

        errors = []

        # Build signal name -> capacity lookup
        signal_lookup: dict[str, int] = {}
        for sig in proton_data.get('signals', []):
            name = sig.get('name')
            sig_cap = sig.get('capacity')
            sig_value = sig.get('value')
            sig_type = sig.get('type')
            if name is not None:
                if sig_cap is not None:
                    # Handle hex strings (e.g., '0x1000') or integers
                    if isinstance(sig_cap, str):
                        sig_cap = int(sig_cap, 0)  # auto-detect base (handles 0x prefix)
                    signal_lookup[name] = sig_cap
                # if signal has a default value
                if sig_value is not None:
                    if sig_type == 'bytes':
                        if sig_cap is None:
                            signal_lookup[name] = len(sig_value)
                        elif sig_cap < len(sig_value):
                            errors.append(
                                f'Signal "{name}" has default value longer '
                                f"than capacity: ({len(sig_value)} > {sig_cap})"
                            )
                    elif sig_type == "string":
                        if sig_cap is None or sig_cap == len(sig_value):
                            signal_lookup[name] = len(sig_value) + 1
                        elif sig_cap < len(sig_value):
                            errors.append(
                                f'Signal "{name}" has default value longer '
                                f"than capacity: ({len(sig_value)} > {sig_cap})"
                            )

        for msg in self.messages:
            for mapping in msg.all_mappings:
                if mapping.signal_name in signal_lookup:
                    mapping.signal_capacity = signal_lookup[mapping.signal_name]
                if mapping.proton_index is not None:
                    if mapping.signal_capacity is None:
                        errors.append(
                            f'Signal "{mapping.signal_name}" needs a capacity for '
                            f'"proton.index" (binding "{msg.name}")'
                        )
                    elif mapping.proton_index >= mapping.signal_capacity:
                        errors.append(
                            f'"proton.index" {mapping.proton_index} is outside the capacity '
                            f'{mapping.signal_capacity} of signal "{mapping.signal_name}" '
                            f'(binding "{msg.name}")'
                        )

        return errors

    def validate(self) -> list[str]:
        """Validate configuration and return list of errors."""
        errors = []

        # Check for duplicate binding names
        names = [m.name for m in self.messages]
        seen = set()
        for name in names:
            if name in seen:
                errors.append(f'Duplicate binding name: {name}')
            seen.add(name)

        # Validate data types
        valid_types = {'double', 'float', 'int32',
                       'int64', 'uint32', 'uint64', 'bool', 'string', 'bytes'}
        for msg in self.messages:
            array_paths = set()
            for array in msg.arrays:
                if not re.fullmatch(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*)*', array.ros_path):
                    errors.append(f'Invalid array path "{array.ros_path}" '
                                  f'in binding "{msg.name}"')
                if array.ros_path in array_paths:
                    errors.append(f'Duplicate array path "{array.ros_path}" '
                                  f'in binding "{msg.name}"')
                array_paths.add(array.ros_path)
                for mapping in msg.mappings:
                    if (mapping.ros_path == array.ros_path or
                            mapping.ros_path.startswith(array.ros_path + '[') or
                            mapping.ros_path.startswith(array.ros_path + '.')):
                        errors.append(f'Array path "{array.ros_path}" also has '
                                      f'an explicit mapping '
                                      f'in binding "{msg.name}"')
            for mapping in msg.mappings:
                if mapping.path_error:
                    errors.append(f'{mapping.path_error} (binding "{msg.name}")')
                if mapping.proton_index is not None:
                    index = mapping.proton_index
                    if (
                        isinstance(index, bool)
                        or not isinstance(index, int)
                        or index < 0
                    ):
                        errors.append(
                            f'"proton.index" must be a non-negative integer in binding '
                            f'"{msg.name}", field "{mapping.ros_path}"'
                        )
                    if mapping.data_type != "bytes":
                        errors.append(
                            f'"proton.index" needs a bytes signal in binding "{msg.name}", '
                            f'field "{mapping.ros_path}"'
                        )
                if mapping.data_type not in valid_types:
                    errors.append(
                        f'Invalid data type "{mapping.data_type}" in binding "{msg.name}"'
                    )

        return errors


@dataclass
class PackageConfig:
    """Configuration for the generated package metadata."""

    package_name: str
    version: str = '0.0.1'
    maintainer_name: str = 'Unknown'
    maintainer_email: str = 'unknown@example.com'
    pkg_license: str = 'TODO: License declaration'

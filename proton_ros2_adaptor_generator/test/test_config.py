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

"""Unit tests for proton_ros2_adaptor_generator.config module."""

from pathlib import Path

from proton_ros2_adaptor_generator.config import (
    AdaptorConfig,
    Mapping,
    MappingType,
    MessageBinding,
)


# Path to test fixtures
FIXTURES_DIR = Path(__file__).parent / "fixtures"


class TestMapping:
    """Tests for Mapping dataclass."""

    def test_scalar_mapping_type(self):
        """Scalar mapping (no ros_index) should be SCALAR type."""
        m = Mapping(ros_path="data", signal_name="temp", data_type="float")
        assert m.mapping_type == MappingType.SCALAR
        assert m.ros_index is None

    def test_indexed_mapping_type(self):
        """Mapping with ros_index should be ROS_INDEXED type."""
        m = Mapping(ros_path="temps", signal_name="temp0", data_type="float", ros_index=0)
        assert m.mapping_type == MappingType.ROS_INDEXED
        assert m.ros_index == 0

    def test_from_dict_scalar(self):
        """Parse scalar mapping from dict."""
        d = {"ros2.path": "data", "proton.signal": "value", "type": "float"}
        m = Mapping.from_dict(d)
        assert m.ros_path == "data"
        assert m.signal_name == "value"
        assert m.data_type == "float"
        assert m.ros_index is None

    def test_from_dict_indexed(self):
        """Parse indexed mapping from dict."""
        d = {"ros2.path": "temps", "proton.signal": "temp0", "type": "float", "ros2.index": 0}
        m = Mapping.from_dict(d)
        assert m.ros_path == "temps"
        assert m.signal_name == "temp0"
        assert m.data_type == "float"
        assert m.ros_index == 0


class TestMessageBinding:
    """Tests for MessageBinding dataclass."""

    def test_adaptor_class_name(self):
        """adaptor_class should generate correct C++ class name."""
        binding = MessageBinding(name="BoardTemps", ros2_type="pkg/msg/Temp")
        assert binding.adaptor_class == "BoardTempsAdaptor"

    def test_source_file_camel_case(self):
        """source_file should convert CamelCase to snake_case."""
        binding = MessageBinding(name="BoardTemps", ros2_type="pkg/msg/Temp")
        assert binding.source_file == "board_temps_adaptor.cpp"

    def test_source_file_single_word(self):
        """source_file should handle single word names."""
        binding = MessageBinding(name="Drive", ros2_type="pkg/msg/Drive")
        assert binding.source_file == "drive_adaptor.cpp"

    def test_ros_cpp_type(self):
        """ros_cpp_type should convert slashes to double colons."""
        binding = MessageBinding(name="Test", ros2_type="geometry_msgs/msg/Twist")
        assert binding.ros_cpp_type == "geometry_msgs::msg::Twist"

    def test_hpp_include(self):
        """hpp_include should generate lowercase include path."""
        binding = MessageBinding(name="Test", ros2_type="geometry_msgs/msg/Twist")
        assert binding.hpp_include == "geometry_msgs/msg/twist.hpp"

    def test_ros_package(self):
        """ros_package should extract package name from type."""
        binding = MessageBinding(name="Test", ros2_type="geometry_msgs/msg/Twist")
        assert binding.ros_package == "geometry_msgs"


class TestAdaptorConfig:
    """Tests for AdaptorConfig dataclass."""

    def test_from_yaml_valid(self):
        """Load valid config from YAML."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "valid_config.yaml")
        assert len(config.messages) == 2
        assert config.messages[0].name == "Drive"
        assert config.messages[1].name == "BoardTemps"
        assert len(config.messages[0].mappings) == 3
        assert len(config.messages[1].mappings) == 3

    def test_ros_msg_dependencies(self):
        """ros_msg_dependencies should collect unique package names."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "valid_config.yaml")
        deps = config.ros_msg_dependencies
        assert "geometry_msgs" in deps
        assert "clearpath_platform_msgs" in deps
        assert len(deps) == 2

    def test_validate_invalid_type(self):
        """Validation should catch invalid data types."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "invalid_type.yaml")
        errors = config.validate()
        assert len(errors) == 1
        assert "invalid_type" in errors[0]
        assert "BadType" in errors[0]

    def test_validate_duplicate_names(self):
        """Validation should catch duplicate binding names."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "duplicate_names.yaml")
        errors = config.validate()
        assert len(errors) == 1
        assert "Duplicate" in errors[0]

    def test_validate_valid_config(self):
        """Valid config should have no validation errors."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "valid_config.yaml")
        errors = config.validate()
        assert len(errors) == 0


class TestSignalResolution:
    """Tests for signal ID resolution from proton config."""

    def test_resolve_signal_ids_success(self):
        """Signal IDs should be resolved from proton config."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "valid_config.yaml")
        errors = config.resolve_signal_ids(FIXTURES_DIR / "proton_signals.yaml")
        assert len(errors) == 0

        # Check Drive mappings got resolved
        drive = config.messages[0]
        assert drive.mappings[0].signal_id == 0x1000  # forward_speed
        assert drive.mappings[1].signal_id == 0x1001  # strafe_speed
        assert drive.mappings[2].signal_id == 0x1002  # turn_rate

    def test_resolve_signal_ids_missing(self):
        """Missing signal should return error."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "missing_signal.yaml")
        errors = config.resolve_signal_ids(FIXTURES_DIR / "proton_signals.yaml")
        assert len(errors) == 1
        assert "nonexistent_signal" in errors[0]
        assert "MissingSignal" in errors[0]

    def test_resolve_signal_ids_hex_format(self):
        """Hex signal IDs (0x1000) should be parsed correctly."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / "valid_config.yaml")
        config.resolve_signal_ids(FIXTURES_DIR / "proton_signals.yaml")

        # Verify hex values are converted to integers
        drive = config.messages[0]
        assert drive.mappings[0].signal_id == 4096  # 0x1000
        assert isinstance(drive.mappings[0].signal_id, int)

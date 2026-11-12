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
from proton_ros2_adaptor_generator.generator import create_jinja_env

import pytest

import yaml


# Path to test fixtures
FIXTURES_DIR = Path(__file__).parent / 'fixtures'


class TestMapping:
    """Tests for Mapping dataclass."""

    def test_scalar_mapping_type(self):
        """Scalar mapping (no ros_index) should be SCALAR type."""
        m = Mapping(ros_path='data', signal_name='temp', data_type='float')
        assert m.mapping_type == MappingType.SCALAR
        assert m.ros_index is None

    def test_indexed_mapping_type(self):
        """Mapping with ros_index should be ROS_INDEXED type."""
        m = Mapping(ros_path='temps', signal_name='temp0', data_type='float', ros_index=0)
        assert m.mapping_type == MappingType.ROS_INDEXED
        assert m.ros_index == 0

    def test_from_dict_scalar(self):
        """Parse scalar mapping from dict."""
        d = {'ros2.path': 'data', 'proton.signal': 'value', 'type': 'float'}
        m = Mapping.from_dict(d)
        assert m.ros_path == 'data'
        assert m.signal_name == 'value'
        assert m.data_type == 'float'
        assert m.ros_index is None

    def test_from_dict_indexed(self):
        """Parse indexed mapping from dict."""
        d = {'ros2.path': 'temps', 'proton.signal': 'temp0', 'type': 'float', 'ros2.index': 0}
        m = Mapping.from_dict(d)
        assert m.ros_path == 'temps'
        assert m.signal_name == 'temp0'
        assert m.data_type == 'float'
        assert m.ros_index == 0


class TestMessageBinding:
    """Tests for MessageBinding dataclass."""

    def test_adaptor_class_name(self):
        """adaptor_class should generate correct C++ class name."""
        binding = MessageBinding(name='BoardTemps', ros2_type='pkg/msg/Temp')
        assert binding.adaptor_class == 'BoardTempsAdaptor'

    def test_source_file_camel_case(self):
        """source_file should convert CamelCase to snake_case."""
        binding = MessageBinding(name='BoardTemps', ros2_type='pkg/msg/Temp')
        assert binding.source_file == 'board_temps_adaptor.cpp'

    def test_source_file_single_word(self):
        """source_file should handle single word names."""
        binding = MessageBinding(name='Drive', ros2_type='pkg/msg/Drive')
        assert binding.source_file == 'drive_adaptor.cpp'

    def test_ros_cpp_type(self):
        """ros_cpp_type should convert slashes to double colons."""
        binding = MessageBinding(name='Test', ros2_type='geometry_msgs/msg/Twist')
        assert binding.ros_cpp_type == 'geometry_msgs::msg::Twist'

    def test_hpp_include(self):
        """hpp_include should generate lowercase include path."""
        binding = MessageBinding(name='Test', ros2_type='geometry_msgs/msg/Twist')
        assert binding.hpp_include == 'geometry_msgs/msg/twist.hpp'
        # Deal with PascalCase names
        pascal_case = MessageBinding(name='TestUInt32', ros2_type='std_msgs/msg/UInt32')
        assert pascal_case.hpp_include == 'std_msgs/msg/u_int32.hpp'
        # Deal with user-defined path
        user_defined = MessageBinding(name='UserDefined',
                                      ros2_type='std_msgs/msg/ColorRGBA',
                                      hpp_path='std_msgs/msg/color_rgba.hpp')
        assert user_defined.hpp_include == 'std_msgs/msg/color_rgba.hpp'

    def test_ros_package(self):
        """ros_package should extract package name from type."""
        binding = MessageBinding(name='Test', ros2_type='geometry_msgs/msg/Twist')
        assert binding.ros_package == 'geometry_msgs'


class TestAdaptorConfig:
    """Tests for AdaptorConfig dataclass."""

    def test_from_yaml_valid(self):
        """Load valid config from YAML."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'valid_config.yaml')
        assert len(config.messages) == 2
        assert config.messages[0].name == 'Drive'
        assert config.messages[1].name == 'BoardTemps'
        assert len(config.messages[0].mappings) == 3
        assert len(config.messages[1].mappings) == 3

    def test_ros_msg_dependencies(self):
        """ros_msg_dependencies should collect unique package names."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'valid_config.yaml')
        deps = config.ros_msg_dependencies
        assert 'geometry_msgs' in deps
        assert 'clearpath_platform_msgs' in deps
        assert len(deps) == 2

    def test_validate_invalid_type(self):
        """Validation should catch invalid data types."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'invalid_type.yaml')
        errors = config.validate()
        assert len(errors) == 1
        assert 'invalid_type' in errors[0]
        assert 'BadType' in errors[0]

    def test_validate_duplicate_names(self):
        """Validation should catch duplicate binding names."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'duplicate_names.yaml')
        errors = config.validate()
        assert len(errors) == 1
        assert 'Duplicate' in errors[0]

    def test_validate_valid_config(self):
        """Valid config should have no validation errors."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'valid_config.yaml')
        errors = config.validate()
        assert len(errors) == 0


class TestSignalResolution:
    """Tests for signal ID resolution from proton config."""

    def test_resolve_signal_ids_success(self):
        """Signal IDs should be resolved from proton config."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'valid_config.yaml')
        errors = config.resolve_signal_ids(FIXTURES_DIR / 'proton_signals.yaml')
        assert len(errors) == 0

        # Check Drive mappings got resolved
        drive = config.messages[0]
        assert drive.mappings[0].signal_id == 0x1000  # forward_speed
        assert drive.mappings[1].signal_id == 0x1001  # strafe_speed
        assert drive.mappings[2].signal_id == 0x1002  # turn_rate

    def test_resolve_signal_ids_missing(self):
        """Missing signal should return error."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'missing_signal.yaml')
        errors = config.resolve_signal_ids(FIXTURES_DIR / 'proton_signals.yaml')
        assert len(errors) == 1
        assert 'nonexistent_signal' in errors[0]
        assert 'MissingSignal' in errors[0]

    def test_resolve_signal_ids_hex_format(self):
        """Hex signal IDs (0x1000) should be parsed correctly."""
        config = AdaptorConfig.from_yaml(FIXTURES_DIR / 'valid_config.yaml')
        config.resolve_signal_ids(FIXTURES_DIR / 'proton_signals.yaml')

        # Verify hex values are converted to integers
        drive = config.messages[0]
        assert drive.mappings[0].signal_id == 4096  # 0x1000
        assert isinstance(drive.mappings[0].signal_id, int)


class TestArrayBinding:
    """Variable arrays resolve their count and ordered element signals independently."""

    @pytest.fixture
    def array_config(self):
        """Create a binding with both a variable array and a legacy indexed field."""
        return AdaptorConfig(messages=[MessageBinding.from_dict({
            'name': 'Battery', 'ros2_type': 'sensor_msgs/msg/BatteryState',
            'mapping': [{'ros2.path': 'other', 'ros2.index': 2,
                         'proton.signal': 'first', 'type': 'float'}],
            'arrays': [{'ros2.path': 'cell_voltage', 'elements': ['second', 'first'],
                        'num_elements': 'count'}],
        })])

    def test_parse_and_resolve(self, array_config, tmp_path):
        """Resolve array types and IDs from the schema without reordering signals."""
        path = tmp_path / 'signals.yaml'
        path.write_text(yaml.safe_dump({'signals': [
            {'name': 'first', 'id': 1, 'type': 'float'},
            {'name': 'second', 'id': 2, 'type': 'float'},
            {'name': 'count', 'id': 3, 'type': 'uint32'},
        ]}))
        assert array_config.validate() == []
        assert array_config.resolve_signal_ids(path) == []
        array = array_config.messages[0].arrays[0]
        assert [element.signal_id for element in array.elements] == [2, 1]
        assert [element.data_type for element in array.elements] == ['float', 'float']
        assert array.num_elements.signal_id == 3
        assert array.num_elements.data_type == 'uint32'
        assert array_config.messages[0].mappings[0].ros_index == 2

    def test_missing_signals(self, array_config):
        """Report missing array elements and count signals through existing resolution."""
        errors = array_config.resolve_signal_ids(FIXTURES_DIR / 'proton_signals.yaml')
        assert any('count' in error for error in errors)
        assert any('second' in error for error in errors)

    def test_reject_non_integer_count(self, array_config, tmp_path):
        """A floating-point count is not a valid array length."""
        path = tmp_path / 'signals.yaml'
        path.write_text(yaml.safe_dump({'signals': [
            {'name': 'count', 'id': 3, 'type': 'float'},
        ]}))
        assert any('integer type' in error
                   for error in array_config.resolve_signal_ids(path))

    def test_reject_overlapping_mapping(self, array_config):
        """An explicit index must not silently override an array's length."""
        array_config.messages[0].mappings[0].ros_path = 'cell_voltage'
        assert any('explicit mapping' in error for error in array_config.validate())

    @pytest.mark.parametrize('ros_type,path', [
        ('sensor_msgs/msg/BatteryState', 'cell_voltage'),
        ('sensor_msgs/msg/BatteryState', 'cell_temperature'),
    ])
    def test_render_array_conversions(self, array_config, ros_type, path):
        """Both callbacks guard copies, preserve ordering, and retain legacy mappings."""
        binding = array_config.messages[0]
        binding.ros2_type = ros_type
        array = binding.arrays[0]
        array.ros_path = path
        array.num_elements.data_type = 'uint32'
        array.num_elements.signal_id = 3
        for element in array.elements:
            element.ros_path = path
            element.data_type = 'float'
            element.signal_id = 2 - element.ros_index
        source = create_jinja_env().get_template('adaptor.cpp.jinja').render(binding=binding)
        assert f'msg.{path}.resize(static_cast<size_t>(count))' in source
        assert f'static_cast<uint64_t>(count) > msg.{path}.max_size()' in source
        assert 'count < 0' not in source
        assert f'count_signal.set(static_cast<CountType>(msg.{path}.size()))' in source
        assert source.count(f'if (msg.{path}.size() > 0)') == 2
        assert source.count(f'if (msg.{path}.size() > 1)') == 2
        assert f'set_indexed_field<0>(msg.{path}, tmp)' in source
        assert f'indexed_field<1>(msg.{path})' in source
        assert 'set_indexed_field<2>(msg.other, tmp)' in source
        assert '#include <limits>' in source
        assert '#include <stdexcept>' in source

    def test_empty_array(self, array_config):
        """An empty signal list resizes without generating element accesses."""
        array = array_config.messages[0].arrays[0]
        array.elements = []
        array.num_elements.data_type = 'int32'
        array.num_elements.signal_id = 3
        macros = create_jinja_env().get_template('macros.jinja').module
        outgoing = getattr(macros, 'array_to_ros')(array)
        incoming = getattr(macros, 'array_to_proton')(array)
        assert 'if (count < 0)' in outgoing
        assert 'static_cast<uint64_t>(count) > msg.cell_voltage.max_size()' in outgoing
        assert '.resize(static_cast<size_t>(count))' in outgoing
        assert 'indexed_field' not in outgoing + incoming
        assert 'std::numeric_limits<CountType>::max()' in incoming

    @pytest.mark.parametrize('data_type', ['string', 'bytes'])
    def test_element_capacities(self, array_config, tmp_path, data_type):
        """Variable arrays also resolve capacities for nonnumeric element signals."""
        path = tmp_path / 'signals.yaml'
        path.write_text(yaml.safe_dump({'signals': [
            {'name': 'first', 'id': 1, 'type': data_type, 'capacity': 8},
            {'name': 'second', 'id': 2, 'type': data_type, 'capacity': 16},
            {'name': 'count', 'id': 3, 'type': 'uint64'},
        ]}))
        assert array_config.resolve_signal_ids(path) == []
        assert array_config.resolve_signal_capacities(path) == []
        array = array_config.messages[0].arrays[0]
        assert [element.signal_capacity for element in array.elements] == [16, 8]
        macros = create_jinja_env().get_template('macros.jinja').module
        source = getattr(macros, 'array_to_ros')(array)
        assert 'tmp.resize(16)' in source
        assert 'tmp.resize(8)' in source

    @pytest.mark.parametrize('path', ['cell_voltage', 'cell_voltage[1]', 'cell_voltage[1].x'])
    def test_indexed_path_overlap(self, array_config, path):
        """PR #47 indexed paths cannot resize a count-controlled field independently."""
        array_config.messages[0].mappings[0].ros_path = path
        assert any('explicit mapping' in error for error in array_config.validate())

    def test_duplicate_array_path(self, array_config):
        """Two count signals cannot control the same ROS sequence."""
        binding = array_config.messages[0]
        binding.arrays.append(binding.arrays[0])
        assert any('Duplicate array path' in error for error in array_config.validate())

    def test_invalid_array_path(self, array_config):
        """Array paths identify a sequence, not one indexed element."""
        array_config.messages[0].arrays[0].ros_path = 'cell_voltage[0]'
        assert any('Invalid array path' in error for error in array_config.validate())

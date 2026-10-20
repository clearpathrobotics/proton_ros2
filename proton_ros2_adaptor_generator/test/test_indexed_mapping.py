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

"""Compile and execute generated mappings without requiring a ROS installation."""

from pathlib import Path
import shutil
import subprocess

from proton_ros2_adaptor_generator.config import Mapping, MessageBinding
from proton_ros2_adaptor_generator.generator import create_jinja_env
import pytest


INCLUDE_DIR = Path(__file__).resolve().parents[2] / 'proton_ros2_interfaces' / 'include'

CPP_SUPPORT = """
#include <proton_ros2_interfaces/indexed_field.hpp>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

constexpr int PROTON_OK = 0;
namespace proton {
template<typename T>
struct Signal {
  Signal(void *, int) {}
  inline static T input{};
  inline static T output{};
  int get(T & value) { value = input; return PROTON_OK; }
  void set(const T & value) { output = value; }
};
}

struct GeneratedNode {
  void * registry() { return nullptr; }
};

template<typename T, std::size_t Bound>
struct BoundedVector : std::vector<T> {
  void resize(std::size_t size) {
    if (size > Bound) {
      throw std::length_error("bounded sequence maximum exceeded");
    }
    std::vector<T>::resize(size);
  }
};
"""


@pytest.fixture
def compile_cpp(tmp_path):
    """Compile a standalone generated mapping and optionally execute it."""
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.fail('A C++ compiler is required for indexed mapping regression tests')

    def compile_source(source, *, expect_failure=False):
        source_path = tmp_path / 'mapping.cpp'
        executable = tmp_path / 'mapping'
        source_path.write_text(CPP_SUPPORT + source)
        result = subprocess.run(
            [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
             '-I', str(INCLUDE_DIR), str(source_path), '-o', str(executable)],
            capture_output=True, text=True,
        )
        if expect_failure:
            assert result.returncode != 0
            assert 'Mapped ROS array index is out of bounds' in result.stderr
        else:
            assert result.returncode == 0, result.stderr
            subprocess.run([str(executable)], check=True, capture_output=True, text=True)

    return compile_source


def render_mapping(direction, index, data_type='float'):
    """Render the same macros used in generated publisher/subscriber callbacks."""
    mapping = Mapping(
        ros_path='values', signal_name='value', data_type=data_type,
        ros_index=index, signal_id=10,
    )
    macros = create_jinja_env().get_template('macros.jinja').module
    return getattr(macros, direction)(mapping)


@pytest.mark.parametrize('container', [
    'std::array<double, 9>',
    'std::vector<double>',
    'BoundedVector<double, 9>',
])
def test_indexed_numeric_round_trip(compile_cpp, container):
    """Fixed and dynamic arrays support out-of-order mappings without shrinking."""
    compile_cpp(f"""
int main() {{
  struct {{ {container} values{{}}; }} msg;
  void * registry = nullptr;
  GeneratedNode generated_node;
  proton::Signal<float>::input = 8.5f;
  {render_mapping('to_ros', 8)}
  proton::Signal<float>::input = 0.5f;
  {render_mapping('to_ros', 0)}
  assert(msg.values.size() == 9);
  assert(msg.values[8] == 8.5);
  assert(msg.values[0] == 0.5);
  {render_mapping('to_proton', 8)}
  assert(proton::Signal<float>::output == 8.5f);
}}
""")


@pytest.mark.parametrize('container', [
    'std::array<std::string, 9>',
    'std::vector<std::string>',
])
def test_indexed_string_subscription(compile_cpp, container):
    """Non-numeric subscriber mappings also use checked array access."""
    compile_cpp(f"""
int main() {{
  struct {{ {container} values{{}}; }} msg;
  GeneratedNode generated_node;
  proton_ros2_interfaces::set_indexed_field<8>(msg.values, std::string("imu"));
  {render_mapping('to_proton', 8, 'string')}
  assert(proton::Signal<std::string>::output == "imu");
}}
""")


def test_short_subscription_array(compile_cpp):
    """Short incoming sequences raise an error instead of reading invalid memory."""
    compile_cpp(f"""
int main() {{
  struct {{ std::vector<double> values; }} msg;
  GeneratedNode generated_node;
  proton::Signal<float>::output = 42.0f;
  bool threw = false;
  try {{
    {render_mapping('to_proton', 8)}
  }} catch (const std::out_of_range &) {{
    threw = true;
  }}
  assert(threw);
  assert(proton::Signal<float>::output == 42.0f);
}}
""")


def test_bounded_sequence_overflow(compile_cpp):
    """Publisher mappings preserve a bounded sequence's size enforcement."""
    compile_cpp(f"""
int main() {{
  struct {{ BoundedVector<double, 9> values; }} msg;
  void * registry = nullptr;
  bool threw = false;
  try {{
    {render_mapping('to_ros', 9)}
  }} catch (const std::length_error &) {{
    threw = true;
  }}
  assert(threw);
}}
""")


@pytest.mark.parametrize('direction', ['to_ros', 'to_proton'])
def test_invalid_fixed_array_index(compile_cpp, direction):
    """Both conversion directions reject invalid fixed-array indices at compile time."""
    compile_cpp(f"""
int main() {{
  struct {{ std::array<double, 9> values{{}}; }} msg;
  [[maybe_unused]] void * registry = nullptr;
  [[maybe_unused]] GeneratedNode generated_node;
  {render_mapping(direction, 9)}
}}
""", expect_failure=True)


def test_adaptor_includes_indexed_field_helper():
    """Complete adaptor sources include the header required by indexed mappings."""
    binding = MessageBinding(
        name='Imu', ros2_type='sensor_msgs/msg/Imu',
        mappings=[Mapping(
            ros_path='orientation_covariance', signal_name='variance',
            data_type='float', ros_index=8, signal_id=10,
        )],
    )
    source = create_jinja_env().get_template('adaptor.cpp.jinja').render(binding=binding)
    assert '#include <proton_ros2_interfaces/indexed_field.hpp>' in source
    assert 'set_indexed_field<8>(msg.orientation_covariance, tmp)' in source
    assert 'indexed_field<8>(msg.orientation_covariance)' in source
    assert 'orientation_covariance.resize' not in source

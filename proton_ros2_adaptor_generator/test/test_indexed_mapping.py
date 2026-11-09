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

import shutil
import subprocess
from pathlib import Path

import pytest
import yaml
from proton_ros2_adaptor_generator.config import AdaptorConfig, Mapping, MessageBinding
from proton_ros2_adaptor_generator.generator import create_jinja_env

INCLUDE_DIR = Path(__file__).resolve().parents[2] / "proton_ros2_interfaces" / "include"

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


RGB_STRUCT = "struct Rgb { uint8_t red{}; uint8_t green{}; uint8_t blue{}; };"
BYTES = "std::vector<uint8_t>"


def render_path_mapping(direction, path, data_type, capacity=None, proton_index=None):
    """Render a mapping whose ros2.path may carry array indexes anywhere."""
    mapping = Mapping(
        ros_path=path,
        signal_name="value",
        data_type=data_type,
        signal_id=10,
        signal_capacity=capacity,
        proton_index=proton_index,
    )
    macros = create_jinja_env().get_template("macros.jinja").module
    return getattr(macros, direction)(mapping)


@pytest.mark.parametrize(
    "container",
    [
        "std::array<Rgb, 4>",
        "std::vector<Rgb>",
    ],
)
def test_a_byte_of_a_signal_fills_one_member(compile_cpp, container):
    """One byte of a bytes signal is written to one member of an array element."""
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ {container} values{{}}; }} msg;
  void * registry = nullptr;
  proton::Signal<{BYTES}>::input = {{10, 20, 30}};
  {render_path_mapping("to_ros", "values[3].green", "bytes", 3, 1)}
  assert(msg.values.size() == 4);
  assert(msg.values[3].green == 20);
  assert(msg.values[3].red == 0);
  assert(msg.values[3].blue == 0);
}}
""")


def test_a_member_replaces_one_byte_and_keeps_the_others(compile_cpp):
    """The rest of the signal keeps its current value."""
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::vector<Rgb> values; }} msg;
  msg.values.resize(4);
  msg.values[3].green = 99;
  GeneratedNode generated_node;
  proton::Signal<{BYTES}>::input = {{10, 20, 30}};
  {render_path_mapping("to_proton", "values[3].green", "bytes", 3, 1)}
  assert((proton::Signal<{BYTES}>::output == {BYTES}{{10, 99, 30}}));
}}
""")


def test_three_members_make_one_signal(compile_cpp):
    """Mappings that share a signal build it up one byte at a time, in both directions."""
    signal = f"proton::Signal<{BYTES}>"
    to_proton = f"\n  {signal}::input = {signal}::output;\n  ".join(
        render_path_mapping("to_proton", f"values[0].{member}", "bytes", 3, index)
        for index, member in enumerate(("red", "green", "blue"))
    )
    to_ros = "\n  ".join(
        render_path_mapping("to_ros", f"values[0].{member}", "bytes", 3, index)
        for index, member in enumerate(("red", "green", "blue"))
    )
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::vector<Rgb> values; }} msg;
  void * registry = nullptr;
  GeneratedNode generated_node;
  proton::Signal<{BYTES}>::input = {{10, 20, 30}};
  {to_ros}
  assert(msg.values[0].red == 10 && msg.values[0].green == 20 && msg.values[0].blue == 30);
  msg.values[0].red = 1;
  msg.values[0].green = 2;
  msg.values[0].blue = 3;
  {to_proton}
  assert((proton::Signal<{BYTES}>::output == {BYTES}{{1, 2, 3}}));
}}
""")


def test_short_array_leaves_the_signal_alone(compile_cpp):
    """A missing element raises an error before anything is written."""
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::vector<Rgb> values; }} msg;
  msg.values.resize(1);
  GeneratedNode generated_node;
  proton::Signal<{BYTES}>::input = {{10, 20, 30}};
  proton::Signal<{BYTES}>::output = {{7, 7, 7}};
  bool threw = false;
  try {{
    {render_path_mapping("to_proton", "values[3].red", "bytes", 3, 0)}
  }} catch (const std::out_of_range &) {{
    threw = true;
  }}
  assert(threw);
  assert((proton::Signal<{BYTES}>::output == {BYTES}{{7, 7, 7}}));
}}
""")


@pytest.mark.parametrize("direction", ["to_ros", "to_proton"])
def test_invalid_fixed_array_index_with_a_signal_byte(compile_cpp, direction):
    """Bounds are still checked at compile time when a byte is selected."""
    compile_cpp(
        f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::array<Rgb, 4> values{{}}; }} msg;
  [[maybe_unused]] void * registry = nullptr;
  [[maybe_unused]] GeneratedNode generated_node;
  {render_path_mapping(direction, "values[4].red", "bytes", 3, 0)}
}}
""",
        expect_failure=True,
    )


def test_adaptor_uses_one_byte_of_the_signal():
    """Complete adaptor sources read the byte in one direction and replace it in the other."""
    binding = MessageBinding(
        name="Lights",
        ros2_type="clearpath_platform_msgs/msg/Lights",
        mappings=[
            Mapping(
                ros_path="lights[1].red",
                signal_name="light",
                data_type="bytes",
                proton_index=0,
                signal_id=10,
                signal_capacity=3,
            )
        ],
    )
    source = (
        create_jinja_env().get_template("adaptor.cpp.jinja").render(binding=binding)
    )
    assert "indexed_element<1>(msg.lights).red = tmp[0];" in source
    assert "std::vector<uint8_t> val(3);" in source
    assert "val[0] = static_cast<uint8_t>(" in source


def test_proton_index_is_parsed_from_the_mapping():
    """The mapping key is proton.index."""
    mapping = Mapping.from_dict(
        {
            "ros2.path": "lights[1].red",
            "proton.signal": "light",
            "proton.index": 2,
            "type": "bytes",
        }
    )
    assert mapping.proton_index == 2


def _config_with(proton_index, data_type="bytes"):
    return AdaptorConfig(
        messages=[
            MessageBinding(
                name="Lights",
                ros2_type="clearpath_platform_msgs/msg/Lights",
                mappings=[
                    Mapping(
                        ros_path="lights[0].red",
                        signal_name="light",
                        data_type=data_type,
                        proton_index=proton_index,
                    )
                ],
            )
        ]
    )


@pytest.mark.parametrize("index", [-1, "x", True, 1.5])
def test_proton_index_must_be_a_non_negative_integer(index):
    """Anything but a whole number from zero up is rejected."""
    assert any("proton.index" in error for error in _config_with(index).validate())


def test_proton_index_needs_a_bytes_signal():
    """Only a bytes signal has bytes to choose from."""
    errors = _config_with(0, data_type="uint32").validate()
    assert any("bytes signal" in error for error in errors)


@pytest.mark.parametrize(
    "capacity, index, accepted",
    [
        (3, 0, True),
        (3, 2, True),
        (3, 3, False),
        (None, 0, False),
    ],
)
def test_proton_index_must_be_inside_the_signal(tmp_path, capacity, index, accepted):
    """The index has to be below the signal's capacity."""
    signal = {"id": 10, "name": "light", "type": "bytes"}
    if capacity is not None:
        signal["capacity"] = capacity
    proton_config = tmp_path / "proton.yaml"
    proton_config.write_text(yaml.safe_dump({"signals": [signal]}))
    errors = _config_with(index).resolve_signal_capacities(proton_config)
    assert (errors == []) == accepted


@pytest.mark.parametrize("container", ["std::array<Rgb, 4>", "std::vector<Rgb>"])
def test_one_member_of_an_array_element(compile_cpp, container):
    """A single member of an element maps to a scalar signal and leaves the others alone."""
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ {container} values{{}}; }} msg;
  void * registry = nullptr;
  GeneratedNode generated_node;
  proton::Signal<uint32_t>::input = 200;
  {render_path_mapping("to_ros", "values[3].green", "uint32")}
  assert(msg.values.size() == 4);
  assert(msg.values[3].green == 200);
  assert(msg.values[3].red == 0);
  assert(msg.values[3].blue == 0);
  {render_path_mapping("to_proton", "values[3].green", "uint32")}
  assert(proton::Signal<uint32_t>::output == 200);
}}
""")


def test_indexes_at_several_levels(compile_cpp):
    """Arrays of structs that hold arrays of structs, below a plain struct."""
    compile_cpp(f"""
#include <array>
#include <cstdint>
struct Inner {{ double x{{}}; }};
struct Outer {{ std::array<Inner, 2> items{{}}; }};
int main() {{
  struct {{ std::vector<Outer> values{{}}; Outer top{{}}; }} msg;
  void * registry = nullptr;
  GeneratedNode generated_node;
  proton::Signal<double>::input = 1.5;
  {render_path_mapping("to_ros", "values[2].items[1].x", "double")}
  assert(msg.values.size() == 3);
  assert(msg.values[2].items[1].x == 1.5);
  assert(msg.values[2].items[0].x == 0.0);
  {render_path_mapping("to_ros", "top.items[1].x", "double")}
  assert(msg.top.items[1].x == 1.5);
  proton::Signal<double>::output = 0.0;
  {render_path_mapping("to_proton", "values[2].items[1].x", "double")}
  assert(proton::Signal<double>::output == 1.5);
}}
""")


def test_string_and_bytes_members(compile_cpp):
    """Capacity types also work on members of array elements."""
    compile_cpp(f"""
#include <cstdint>
struct Named {{ std::string label; std::vector<uint8_t> data; }};
int main() {{
  struct {{ std::vector<Named> values{{}}; }} msg;
  void * registry = nullptr;
  GeneratedNode generated_node;
  proton::Signal<std::string>::input = "imu";
  {render_path_mapping("to_ros", "values[1].label", "string", 3)}
  assert(msg.values.size() == 2);
  assert(msg.values[1].label == "imu");
  proton::Signal<std::vector<uint8_t>>::input = {{7, 8}};
  {render_path_mapping("to_ros", "values[0].data", "bytes", 2)}
  assert((msg.values[0].data == std::vector<uint8_t>{{7, 8}}));
  {render_path_mapping("to_proton", "values[0].data", "bytes", 2)}
  assert((proton::Signal<std::vector<uint8_t>>::output == std::vector<uint8_t>{{7, 8}}));
  {render_path_mapping("to_proton", "values[1].label", "string", 3)}
  assert(proton::Signal<std::string>::output == "imu");
}}
""")


def test_short_array_for_a_member_mapping(compile_cpp):
    """A short incoming sequence raises an error instead of reading invalid memory."""
    compile_cpp(f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::vector<Rgb> values; }} msg;
  msg.values.resize(1);
  GeneratedNode generated_node;
  bool threw = false;
  try {{
    {render_path_mapping("to_proton", "values[3].red", "uint32")}
  }} catch (const std::out_of_range &) {{
    threw = true;
  }}
  assert(threw);
}}
""")


@pytest.mark.parametrize("direction", ["to_ros", "to_proton"])
def test_invalid_fixed_array_index_in_the_middle_of_a_path(compile_cpp, direction):
    """Every indexed level of a path is bounds-checked at compile time."""
    compile_cpp(
        f"""
#include <cstdint>
{RGB_STRUCT}
int main() {{
  struct {{ std::array<Rgb, 4> values{{}}; }} msg;
  [[maybe_unused]] void * registry = nullptr;
  [[maybe_unused]] GeneratedNode generated_node;
  {render_path_mapping(direction, "values[4].red", "uint32")}
}}
""",
        expect_failure=True,
    )


def test_trailing_index_in_the_path_matches_ros2_index():
    """A trailing index in the path generates the same code as ros2.index."""
    bracket = Mapping(
        ros_path="values[8]", signal_name="value", data_type="float", signal_id=10
    )
    legacy = Mapping(
        ros_path="values",
        signal_name="value",
        data_type="float",
        ros_index=8,
        signal_id=10,
    )
    assert (bracket.ros_path, bracket.ros_index) == ("values", 8)
    macros = create_jinja_env().get_template("macros.jinja").module
    for direction in ("to_ros", "to_proton"):
        assert getattr(macros, direction)(bracket) == getattr(macros, direction)(legacy)


def test_member_of_an_element_is_parsed_from_the_mapping():
    """The index sits inside ros2.path, next to the member it belongs to."""
    mapping = Mapping.from_dict(
        {
            "ros2.path": "lights[1].red",
            "proton.signal": "light_r",
            "type": "uint32",
        }
    )
    assert mapping.nested_access
    assert (
        mapping.write_expr
        == "proton_ros2_interfaces::indexed_element<1>(msg.lights).red"
    )
    assert (
        mapping.read_expr == "proton_ros2_interfaces::indexed_field<1>(msg.lights).red"
    )


@pytest.mark.parametrize(
    "path, index",
    [
        ("values[0", None),
        ("values[]", None),
        ("values..x", None),
        ("1values", None),
        ("values[0].x", 1),
        ("values[0]", 1),
    ],
)
def test_bad_paths_are_rejected(path, index):
    """Malformed paths, and an index given twice, are reported before generating."""
    config = AdaptorConfig(
        messages=[
            MessageBinding(
                name="Bad",
                ros2_type="pkg/msg/Bad",
                mappings=[
                    Mapping(
                        ros_path=path,
                        signal_name="s",
                        data_type="uint32",
                        ros_index=index,
                    )
                ],
            )
        ]
    )
    assert any(
        "ros2.path" in error or "ros2.index" in error for error in config.validate()
    )


def test_adaptor_writes_a_member_of_an_element():
    """Complete adaptor sources use the nested accessors in both directions."""
    binding = MessageBinding(
        name="Lights",
        ros2_type="clearpath_platform_msgs/msg/Lights",
        mappings=[
            Mapping(
                ros_path="lights[1].red",
                signal_name="light_r",
                data_type="uint32",
                signal_id=10,
            )
        ],
    )
    source = (
        create_jinja_env().get_template("adaptor.cpp.jinja").render(binding=binding)
    )
    assert "indexed_element<1>(msg.lights).red = tmp;" in source
    assert "static_cast<uint32_t>(" in source
    assert "proton_ros2_interfaces::indexed_field<1>(msg.lights).red)" in source

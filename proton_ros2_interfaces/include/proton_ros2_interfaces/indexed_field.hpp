/*
 * Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PROTON_ROS2_INTERFACES_INDEXED_FIELD_HPP
#define PROTON_ROS2_INTERFACES_INDEXED_FIELD_HPP

#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>

namespace proton_ros2_interfaces
{

template<typename T>
struct IsFixedArray : std::false_type {};

template<typename T, std::size_t Size>
struct IsFixedArray<std::array<T, Size>>: std::true_type {};

template<std::size_t Index, typename Container>
decltype(auto) indexed_field(Container & field)
{
  using Field = std::remove_cv_t<Container>;
  if constexpr (IsFixedArray<Field>::value) {
    static_assert(
      Index < std::tuple_size<Field>::value,
      "Mapped ROS array index is out of bounds");
  }
  return field.at(Index);
}

template<std::size_t Index, typename Container, typename Value>
void set_indexed_field(Container & field, const Value & value)
{
  if constexpr (!IsFixedArray<Container>::value) {
    if (field.size() <= Index) {
      field.resize(Index + 1);
    }
  }
  indexed_field<Index>(field) = value;
}

}  // namespace proton_ros2_interfaces

#endif  // PROTON_ROS2_INTERFACES_INDEXED_FIELD_HPP

/**
 * @author Levi Armstrong
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_task_composer/core/task_composer_node_ports.h>
#include <tesseract_common/serialization.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
std::string TaskComposerNodePorts::toString() const
{
  std::string supported_ports;
  supported_ports.append("ports:\n");
  supported_ports.append("  inputs:\n");
  if (input_required.empty())
  {
    supported_ports.append("    required: Empty\n");
  }
  else
  {
    std::string temp = "    required: [";
    std::size_t cnt{ 0 };
    for (const auto& [port, type] : input_required)
    {
      std::string bool_str = (static_cast<bool>(type)) ? "Multiple" : "Single";
      temp.append(port);
      temp.append(":");
      temp.append(bool_str);
      if (cnt < input_required.size() - 1)
        temp.append(", ");

      ++cnt;
    }
    temp.append("]\n");

    supported_ports.append(temp);
  }

  if (input_optional.empty())
  {
    supported_ports.append("    optional: Empty\n");
  }
  else
  {
    std::string temp = "    optional: [";
    std::size_t cnt{ 0 };
    for (const auto& [port, type] : input_optional)
    {
      std::string bool_str = (static_cast<bool>(type)) ? "Multiple" : "Single";
      temp.append(port);
      temp.append(":");
      temp.append(bool_str);
      if (cnt < input_optional.size() - 1)
        temp.append(", ");

      ++cnt;
    }
    temp.append("]\n");

    supported_ports.append(temp);
  }

  supported_ports.append("  outputs:\n");
  if (output_required.empty())
  {
    supported_ports.append("    required: Empty\n");
  }
  else
  {
    std::string temp = "    required: [";
    std::size_t cnt{ 0 };
    for (const auto& [port, type] : output_required)
    {
      std::string bool_str = (static_cast<bool>(type)) ? "Multiple" : "Single";
      temp.append(port);
      temp.append(":");
      temp.append(bool_str);
      if (cnt < output_required.size() - 1)
        temp.append(", ");

      ++cnt;
    }
    temp.append("]\n");

    supported_ports.append(temp);
  }

  if (output_optional.empty())
  {
    supported_ports.append("    optional: Empty\n");
  }
  else
  {
    std::string temp = "    optional: [";
    std::size_t cnt{ 0 };
    for (const auto& [port, type] : output_optional)
    {
      std::string bool_str = (static_cast<bool>(type)) ? "Multiple" : "Single";
      temp.append(port);
      temp.append(":");
      temp.append(bool_str);
      if (cnt < output_optional.size() - 1)
        temp.append(", ");

      ++cnt;
    }
    temp.append("]\n");

    supported_ports.append(temp);
  }

  return supported_ports;
}

bool TaskComposerNodePorts::operator==(const TaskComposerNodePorts& rhs) const
{
  bool equal = true;
  equal &= input_required == rhs.input_required;
  equal &= input_optional == rhs.input_optional;
  equal &= output_required == rhs.output_required;
  equal &= output_optional == rhs.output_optional;
  return equal;
}
bool TaskComposerNodePorts::operator!=(const TaskComposerNodePorts& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerNodePorts::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_required);
  ar& BOOST_SERIALIZATION_NVP(input_optional);
  ar& BOOST_SERIALIZATION_NVP(output_required);
  ar& BOOST_SERIALIZATION_NVP(output_optional);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNodePorts)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNodePorts)

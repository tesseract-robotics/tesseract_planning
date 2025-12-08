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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_PORTS_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_PORTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
/**
 * @brief A data structure for holding the nodes required and option input and output ports
 * @details This is leveraged primarily by the validation method.
 * The bool indicates if the port container a container of keys
 */
struct TaskComposerNodePorts
{
  enum Type
  {
    SINGLE = 0,
    MULTIPLE = 1
  };

  std::unordered_map<std::string, Type> input_required;
  std::unordered_map<std::string, Type> input_optional;

  std::unordered_map<std::string, Type> output_required;
  std::unordered_map<std::string, Type> output_optional;

  std::string toString() const;

  bool operator==(const TaskComposerNodePorts& rhs) const;
  bool operator!=(const TaskComposerNodePorts& rhs) const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_PORTS_H

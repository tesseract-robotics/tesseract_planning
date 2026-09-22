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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cstdint>
#include <map>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>

namespace tesseract::task_composer
{
class TaskComposerPortMap;

/** @brief Defines the complete input and output port contract for a fixed task. */
class TaskComposerNodePorts
{
public:
  enum class Cardinality : std::uint8_t
  {
    SINGLE,
    MULTIPLE
  };

  enum class Requirement : std::uint8_t
  {
    REQUIRED,
    OPTIONAL
  };

  enum class Direction : std::uint8_t
  {
    INPUT,
    OUTPUT
  };

  struct PortDefinition
  {
    Cardinality cardinality{ Cardinality::SINGLE };
    Requirement requirement{ Requirement::REQUIRED };

    bool operator==(const PortDefinition& rhs) const;
    bool operator!=(const PortDefinition& rhs) const;
  };

  struct ValidationError
  {
    Direction direction{ Direction::INPUT };
    std::string port;
    std::string message;

    std::string path() const;
    std::string toString() const;
  };

  using ContainerType = std::map<std::string, PortDefinition>;
  using ValidationErrors = std::vector<ValidationError>;

  TaskComposerNodePorts& addRequiredInput(std::string name, Cardinality cardinality = Cardinality::SINGLE);
  TaskComposerNodePorts& addOptionalInput(std::string name, Cardinality cardinality = Cardinality::SINGLE);
  TaskComposerNodePorts& addRequiredOutput(std::string name, Cardinality cardinality = Cardinality::SINGLE);
  TaskComposerNodePorts& addOptionalOutput(std::string name, Cardinality cardinality = Cardinality::SINGLE);

  const ContainerType& inputs() const;
  const ContainerType& outputs() const;

  tesseract::common::PropertyTree inputSchema() const;
  tesseract::common::PropertyTree outputSchema() const;

  ValidationErrors validate(const TaskComposerPortMap& inputs, const TaskComposerPortMap& outputs) const;
  void validateOrThrow(const TaskComposerPortMap& inputs,
                       const TaskComposerPortMap& outputs,
                       const std::string& node_name = {}) const;

  std::string toString() const;

  bool operator==(const TaskComposerNodePorts& rhs) const;
  bool operator!=(const TaskComposerNodePorts& rhs) const;

private:
  template <class Archive>
  friend void serialize(Archive& ar, TaskComposerNodePorts& obj);

  TaskComposerNodePorts& add(Direction direction, Requirement requirement, std::string name, Cardinality cardinality);

  ContainerType input_ports_;
  ContainerType output_ports_;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_PORTS_H

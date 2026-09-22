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

#include <tesseract/task_composer/task_composer_node_ports.h>
#include <tesseract/task_composer/task_composer_port_map.h>
#include <tesseract/common/property_tree.h>

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <string_view>

namespace tesseract::task_composer
{
namespace
{
void validateMultiplePort(const tesseract::common::PropertyTree& node,
                          const std::string& path,
                          std::vector<std::string>& errors)
{
  const YAML::Node& value = node.getValue();
  if (!value || !value.IsSequence())
    return;

  if (value.size() == 0)
    errors.push_back(path + ": port key list must not be empty");

  for (std::size_t index = 0; index < value.size(); ++index)
  {
    if (!value[index].IsScalar() || value[index].Scalar().empty())
      errors.push_back(path + "[" + std::to_string(index) + "]: expected a non-empty string");
  }
}

void validateSinglePort(const tesseract::common::PropertyTree& node,
                        const std::string& path,
                        std::vector<std::string>& errors)
{
  const YAML::Node& value = node.getValue();
  if (value && value.IsScalar() && value.Scalar().empty())
    errors.push_back(path + ": port key must not be empty");
}

void addPort(tesseract::common::PropertyTree& schema,
             std::string_view name,
             TaskComposerNodePorts::Cardinality cardinality,
             bool required)
{
  using namespace tesseract::common;
  auto& port = schema[std::string(name)];
  if (cardinality == TaskComposerNodePorts::Cardinality::MULTIPLE)
  {
    port.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    port.addValidator(validateMultiplePort);
  }
  else
  {
    port.setAttribute(property_attribute::TYPE, property_type::STRING);
    port.addValidator(validateSinglePort);
  }

  if (required)
    port.setAttribute(property_attribute::REQUIRED, true);
}

tesseract::common::PropertyTree createSchema(const TaskComposerNodePorts::ContainerType& definitions)
{
  using namespace tesseract::common;
  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);

  for (const auto& [name, definition] : definitions)
  {
    const bool required = definition.requirement == TaskComposerNodePorts::Requirement::REQUIRED;
    if (required)
      schema.setAttribute(property_attribute::REQUIRED, true);
    addPort(schema, name, definition.cardinality, required);
  }
  return schema;
}

void validateDirection(const TaskComposerNodePorts::ContainerType& definitions,
                       const TaskComposerPortMap& mappings,
                       TaskComposerNodePorts::Direction direction,
                       TaskComposerNodePorts::ValidationErrors& errors)
{
  for (const auto& [name, definition] : definitions)
  {
    if (definition.requirement == TaskComposerNodePorts::Requirement::REQUIRED && !mappings.contains(name))
      errors.push_back({ direction, name, "required port is missing" });
  }

  for (const auto& [name, mapping] : mappings.data())
  {
    const auto definition = definitions.find(name);
    if (definition == definitions.end())
    {
      errors.push_back({ direction, name, "undeclared port" });
      continue;
    }

    const auto cardinality = definition->second.cardinality;
    if (cardinality == TaskComposerNodePorts::Cardinality::SINGLE)
    {
      if (!std::holds_alternative<std::string>(mapping))
      {
        errors.push_back({ direction, name, "expected a single key" });
        continue;
      }
      if (std::get<std::string>(mapping).empty())
        errors.push_back({ direction, name, "key must not be empty" });
    }
    else
    {
      if (!std::holds_alternative<std::vector<std::string>>(mapping))
      {
        errors.push_back({ direction, name, "expected a list of keys" });
        continue;
      }

      const auto& keys = std::get<std::vector<std::string>>(mapping);
      if (keys.empty())
        errors.push_back({ direction, name, "key list must not be empty" });
      for (std::size_t index = 0; index < keys.size(); ++index)
      {
        if (keys[index].empty())
          errors.push_back({ direction, name + "[" + std::to_string(index) + "]", "key must not be empty" });
      }
    }
  }
}

void appendDefinitions(std::ostringstream& output,
                       const TaskComposerNodePorts::ContainerType& definitions,
                       TaskComposerNodePorts::Requirement requirement,
                       std::string_view label)
{
  output << "    " << label << ": ";
  bool first{ true };
  for (const auto& [name, definition] : definitions)
  {
    if (definition.requirement != requirement)
      continue;

    output << (first ? "[" : ", ") << name << ":"
           << (definition.cardinality == TaskComposerNodePorts::Cardinality::MULTIPLE ? "Multiple" : "Single");
    first = false;
  }
  output << (first ? "Empty" : "]") << "\n";
}
}  // namespace

TaskComposerNodePorts& TaskComposerNodePorts::addRequiredInput(std::string name, Cardinality cardinality)
{
  return add(Direction::INPUT, Requirement::REQUIRED, std::move(name), cardinality);
}

TaskComposerNodePorts& TaskComposerNodePorts::addOptionalInput(std::string name, Cardinality cardinality)
{
  return add(Direction::INPUT, Requirement::OPTIONAL, std::move(name), cardinality);
}

TaskComposerNodePorts& TaskComposerNodePorts::addRequiredOutput(std::string name, Cardinality cardinality)
{
  return add(Direction::OUTPUT, Requirement::REQUIRED, std::move(name), cardinality);
}

TaskComposerNodePorts& TaskComposerNodePorts::addOptionalOutput(std::string name, Cardinality cardinality)
{
  return add(Direction::OUTPUT, Requirement::OPTIONAL, std::move(name), cardinality);
}

TaskComposerNodePorts&
TaskComposerNodePorts::add(Direction direction, Requirement requirement, std::string name, Cardinality cardinality)
{
  if (name.empty())
    throw std::invalid_argument("Port name must not be empty");

  auto& ports = direction == Direction::INPUT ? input_ports_ : output_ports_;
  if (ports.find(name) != ports.end())
    throw std::invalid_argument("Port '" + name + "' is already declared for this direction");
  ports.emplace(std::move(name), PortDefinition{ cardinality, requirement });

  return *this;
}

const TaskComposerNodePorts::ContainerType& TaskComposerNodePorts::inputs() const { return input_ports_; }
const TaskComposerNodePorts::ContainerType& TaskComposerNodePorts::outputs() const { return output_ports_; }

tesseract::common::PropertyTree TaskComposerNodePorts::inputSchema() const { return createSchema(input_ports_); }

tesseract::common::PropertyTree TaskComposerNodePorts::outputSchema() const { return createSchema(output_ports_); }

TaskComposerNodePorts::ValidationErrors TaskComposerNodePorts::validate(const TaskComposerPortMap& inputs,
                                                                        const TaskComposerPortMap& outputs) const
{
  ValidationErrors errors;
  validateDirection(input_ports_, inputs, Direction::INPUT, errors);
  validateDirection(output_ports_, outputs, Direction::OUTPUT, errors);
  return errors;
}

void TaskComposerNodePorts::validateOrThrow(const TaskComposerPortMap& inputs,
                                            const TaskComposerPortMap& outputs,
                                            const std::string& node_name) const
{
  const auto errors = validate(inputs, outputs);
  if (errors.empty())
    return;

  std::ostringstream message;
  message << "Task Composer Node '" << node_name << "' has invalid port mappings:";
  for (const auto& error : errors)
    message << "\n- " << error.toString();
  message << "\n" << toString();
  throw std::runtime_error(message.str());
}

std::string TaskComposerNodePorts::toString() const
{
  std::ostringstream output;
  output << "ports:\n  inputs:\n";
  appendDefinitions(output, input_ports_, Requirement::REQUIRED, "required");
  appendDefinitions(output, input_ports_, Requirement::OPTIONAL, "optional");
  output << "  outputs:\n";
  appendDefinitions(output, output_ports_, Requirement::REQUIRED, "required");
  appendDefinitions(output, output_ports_, Requirement::OPTIONAL, "optional");
  return output.str();
}

bool TaskComposerNodePorts::operator==(const TaskComposerNodePorts& rhs) const
{
  return input_ports_ == rhs.input_ports_ && output_ports_ == rhs.output_ports_;
}
bool TaskComposerNodePorts::operator!=(const TaskComposerNodePorts& rhs) const { return !operator==(rhs); }

bool TaskComposerNodePorts::PortDefinition::operator==(const PortDefinition& rhs) const
{
  return cardinality == rhs.cardinality && requirement == rhs.requirement;
}

bool TaskComposerNodePorts::PortDefinition::operator!=(const PortDefinition& rhs) const { return !operator==(rhs); }

std::string TaskComposerNodePorts::ValidationError::path() const
{
  return std::string(direction == Direction::INPUT ? "inputs." : "outputs.") + port;
}

std::string TaskComposerNodePorts::ValidationError::toString() const { return path() + ": " + message; }

}  // namespace tesseract::task_composer

/**
 * @file task_composer_schema.cpp
 * @brief Task composer schema utils
 *
 * @author Levi Armstrong
 * @date June 20. 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#include <tesseract_task_composer/core/task_composer_schema.h>
#include <tesseract_common/property_tree.h>

using namespace tesseract_common;

namespace tesseract_planning
{
tesseract_common::PropertyTree& addConditionalProperty(tesseract_common::PropertyTree& schema, bool default_value)
{
  auto& prop = schema[property_name::CONDITIONAL];
  prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
  prop.setAttribute(property_attribute::DEFAULT, default_value);
  prop.setAttribute(property_attribute::DOC, "Enable conditional execution");
  prop.setAttribute(property_attribute::REQUIRED, false);
  return prop;
}

tesseract_common::PropertyTree& addTriggerAbortProperty(tesseract_common::PropertyTree& schema, bool default_value)
{
  auto& prop = schema[property_name::TRIGGER_ABORT];
  prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
  prop.setAttribute(property_attribute::DEFAULT, default_value);
  prop.setAttribute(property_attribute::DOC, "Indicate if abort should be triggered if this task is reached");
  prop.setAttribute(property_attribute::REQUIRED, false);
  return prop;
}

tesseract_common::PropertyTree& addInputsProperty(tesseract_common::PropertyTree& schema, bool required)
{
  auto& prop = schema[property_name::INPUTS];
  prop.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  prop.setAttribute(property_attribute::DOC, "Input sources");
  prop.setAttribute(property_attribute::REQUIRED, required);
  return prop;
}

tesseract_common::PropertyTree& addOutputsProperty(tesseract_common::PropertyTree& schema, bool required)
{
  auto& prop = schema[property_name::OUTPUTS];
  prop.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  prop.setAttribute(property_attribute::DOC, "Output sources");
  prop.setAttribute(property_attribute::REQUIRED, required);
  return prop;
}

tesseract_common::PropertyTree& addProgramProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema["program"];
  prop.setAttribute(property_attribute::TYPE, property_type::STRING);
  prop.setAttribute(property_attribute::DOC, "The data storage key to locate the program composite instruction");
  prop.setAttribute(property_attribute::REQUIRED, true);
  return prop;
}

tesseract_common::PropertyTree& addEnvironmentProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema["environment"];
  prop.setAttribute(property_attribute::TYPE, property_type::STRING);
  prop.setAttribute(property_attribute::DOC, "The data storage key to locate the environment");
  prop.setAttribute(property_attribute::REQUIRED, true);
  return prop;
}

tesseract_common::PropertyTree& addProfilesProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema["profiles"];
  prop.setAttribute(property_attribute::TYPE, property_type::STRING);
  prop.setAttribute(property_attribute::DOC, "The data storage key to locate the profiles");
  prop.setAttribute(property_attribute::REQUIRED, true);
  return prop;
}

tesseract_common::PropertyTree& addNodesProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema[property_name::NODES];
  prop.setAttribute(property_attribute::TYPE, "TaskComposerGraphNode{}");
  prop.setAttribute(property_attribute::DOC, "Map of all task nodes");
  prop.setAttribute(property_attribute::REQUIRED, true);
  return prop;
}

tesseract_common::PropertyTree& addEdgesProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema[property_name::EDGES];
  prop.setAttribute(property_attribute::TYPE, "TaskComposerGraphEdge[]");
  prop.setAttribute(property_attribute::DOC, "List of graph edges");
  prop.setAttribute(property_attribute::REQUIRED, true);
  prop.addValidator(validateCustomType);
  return prop;
}

tesseract_common::PropertyTree& addTerminalsProperty(tesseract_common::PropertyTree& schema)
{
  auto& prop = schema[property_name::TERMINALS];
  prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
  prop.setAttribute(property_attribute::DOC, "List of terminal tasks");
  prop.setAttribute(property_attribute::REQUIRED, true);
  prop.addValidator(validateTypeCast<std::vector<std::string>>);
  return prop;
}

tesseract_common::PropertyTree getTaskComposerGraphEdgeSchema()
{
  using namespace tesseract_common;
  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::DOC, "TaskComposerGraphEdge");
  {
    auto& prop = schema["source"];
    prop.setAttribute(property_attribute::TYPE, property_type::STRING);
    prop.setAttribute(property_attribute::DOC, "The source task name");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  {
    auto& prop = schema["destinations"];
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "The list of destination task name");
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateTypeCast<std::vector<std::string>>);
  }

  return schema;
}

}  // namespace tesseract_planning

#include <tesseract_common/schema_registration.h>
TESSERACT_REGISTER_SCHEMA(TaskComposerGraphEdge, tesseract_planning::getTaskComposerGraphEdgeSchema)

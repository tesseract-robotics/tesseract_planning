/**
 * @file task_composer_schema.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_SCHEMA_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_SCHEMA_H

#include <tesseract_common/fwd.h>

#include <string_view>

namespace tesseract_common::property_attribute
{
constexpr std::string_view TASK_NAME{ "task_name" };
constexpr std::string_view FACTORY_NAME{ "factory_name" };
}  // namespace tesseract_common::property_attribute

namespace tesseract_planning
{
namespace property_name
{
// General properties
constexpr std::string_view INPUTS{ "inputs" };
constexpr std::string_view OUTPUTS{ "outputs" };
constexpr std::string_view CONDITIONAL{ "conditional" };
constexpr std::string_view TRIGGER_ABORT{ "trigger_abort" };

// Input/Output property names
constexpr std::string_view PROGRAM{ "program" };
constexpr std::string_view ENVIRONMENT{ "environment" };
constexpr std::string_view PROFILES{ "profiles" };

// Graph/Pipeline properties
constexpr std::string_view NODES{ "nodes" };
constexpr std::string_view EDGES{ "edges" };
constexpr std::string_view TERMINALS{ "terminals" };
}  // namespace property_name

tesseract_common::PropertyTree& addConditionalProperty(tesseract_common::PropertyTree& schema,
                                                       bool default_value = true);
tesseract_common::PropertyTree& addTriggerAbortProperty(tesseract_common::PropertyTree& schema,
                                                        bool default_value = false);
tesseract_common::PropertyTree& addInputsProperty(tesseract_common::PropertyTree& schema, bool required = true);
tesseract_common::PropertyTree& addOutputsProperty(tesseract_common::PropertyTree& schema, bool required = true);

tesseract_common::PropertyTree& addProgramProperty(tesseract_common::PropertyTree& schema);
tesseract_common::PropertyTree& addEnvironmentProperty(tesseract_common::PropertyTree& schema);
tesseract_common::PropertyTree& addProfilesProperty(tesseract_common::PropertyTree& schema);

tesseract_common::PropertyTree& addNodesProperty(tesseract_common::PropertyTree& schema);
tesseract_common::PropertyTree& addEdgesProperty(tesseract_common::PropertyTree& schema);
tesseract_common::PropertyTree& addTerminalsProperty(tesseract_common::PropertyTree& schema);

// Schema

tesseract_common::PropertyTree getTaskComposerGraphEdgeSchema();

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_SCHEMA_H

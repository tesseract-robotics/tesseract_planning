/**
 * @file yaml_extensions.cpp
 * @brief YAML Type conversion implementations
 *
 * @author Levi Armstrong
 * @date April 6, 2026
 *
 * @copyright Copyright (c) 2026, Levi Armstrong
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

#include <tesseract/task_composer/yaml_extensions.h>

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>

namespace tesseract::task_composer
{
namespace
{
tesseract::common::PropertyTree createStringOrStringListSchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .oneOf()
        .string("single").done()
        .customType("multiple", property_type::createList(property_type::STRING)).done()
      .build();
  // clang-format on
}

tesseract::common::PropertyTree createRequiredStringOrStringListSchema()
{
  auto schema = createStringOrStringListSchema();
  schema.setAttribute(tesseract::common::property_attribute::REQUIRED, true);
  return schema;
}
}  // namespace
}  // namespace tesseract::task_composer

namespace YAML
{
tesseract::common::PropertyTree convert<tesseract::task_composer::TaskComposerPortMap>::schema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::createMap(tesseract::task_composer::STRING_OR_STRING_LIST_SCHEMA_KEY))
      .build();
  // clang-format on
}
}  // namespace YAML

TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::StringOrStringList,
                          tesseract::task_composer::createStringOrStringListSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::RequiredStringOrStringList,
                          tesseract::task_composer::createRequiredStringOrStringListSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::TaskComposerPortMap,
                          YAML::convert<tesseract::task_composer::TaskComposerPortMap>::schema);

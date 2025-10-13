/**
 * @file yaml_utils.h
 * @brief YAML utility functions
 *
 * @author Levi Armstrong
 * @date October 12, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_CORE_YAML_UTILS_H
#define TESSERACT_TASK_COMPOSER_CORE_YAML_UTILS_H

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TaskComposerNode;
void loadSubTaskConfig(TaskComposerNode& node, const YAML::Node& config);

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_CORE_YAML_UTILS_H

/**
 * @file task_composer_node_types.h
 * @brief A node types enum
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_TYPES_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_TYPES_H

namespace tesseract_planning
{
enum class TaskComposerNodeType
{
  NODE,
  TASK,
  PIPELINE,
  GRAPH
};
}

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_TYPES_H

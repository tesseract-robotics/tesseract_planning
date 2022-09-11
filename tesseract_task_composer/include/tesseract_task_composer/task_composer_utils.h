/**
 * @file task_composer_utils.h
 * @brief A task composer utils
 *
 * @author Levi Armstrong
 * @date August 27, 2022
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_UTILS_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_UTILS_H

#include <tesseract_task_composer/task_composer_server.h>

namespace tesseract_planning
{
void loadDefaultTaskComposerNodes(TaskComposerServer& server);
}

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_UTILS_H

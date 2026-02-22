/**
 * @file update_start_and_end_state_task.h
 *
 * @author Levi Armstrong
 * @date August 5, 2022
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
#ifndef TESSERACT_TASK_COMPOSER_UPDATE_START_AND_END_STATE_TASK_H
#define TESSERACT_TASK_COMPOSER_UPDATE_START_AND_END_STATE_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract::task_composer
{
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT UpdateStartAndEndStateTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_PREVIOUS_PROGRAM_PORT;
  static const std::string INPUT_CURRENT_PROGRAM_PORT;
  static const std::string INPUT_NEXT_PROGRAM_PORT;
  static const std::string OUTPUT_PROGRAM_PORT;

  using Ptr = std::shared_ptr<UpdateStartAndEndStateTask>;
  using ConstPtr = std::shared_ptr<const UpdateStartAndEndStateTask>;
  using UPtr = std::unique_ptr<UpdateStartAndEndStateTask>;
  using ConstUPtr = std::unique_ptr<const UpdateStartAndEndStateTask>;

  UpdateStartAndEndStateTask() = default;
  /** @brief The input_key is the uuid string */
  explicit UpdateStartAndEndStateTask(std::string name,
                                      std::string input_prev_key,
                                      std::string input_next_key,
                                      std::string output_key,
                                      bool conditional);

  explicit UpdateStartAndEndStateTask(std::string name,
                                      std::string input_key,
                                      std::string input_prev_key,
                                      std::string input_next_key,
                                      std::string output_key,
                                      bool conditional);
  ~UpdateStartAndEndStateTask() override = default;

  bool operator==(const UpdateStartAndEndStateTask& rhs) const;
  bool operator!=(const UpdateStartAndEndStateTask& rhs) const;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_UPDATE_START_AND_END_STATE_TASK_H

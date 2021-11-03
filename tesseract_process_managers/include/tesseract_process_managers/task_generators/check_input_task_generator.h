/**
 * @file check_input_task_generator.h
 * @brief Process generator for checking input data structure
 *
 * @author Levi Armstrong
 * @date November 2. 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_PROCESS_MANAGERS_CHECK_INPUT_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_CHECK_INPUT_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
using CheckInputFn = std::function<bool(const TaskInput&)>;

class CheckInputTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<CheckInputTaskGenerator>;

  CheckInputTaskGenerator(std::string name = "Check TaksInput");
  CheckInputTaskGenerator(CheckInputFn fn, std::string name = "Check TaksInput");

  ~CheckInputTaskGenerator() override = default;
  CheckInputTaskGenerator(const CheckInputTaskGenerator&) = delete;
  CheckInputTaskGenerator& operator=(const CheckInputTaskGenerator&) = delete;
  CheckInputTaskGenerator(CheckInputTaskGenerator&&) = delete;
  CheckInputTaskGenerator& operator=(CheckInputTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override;

  void process(TaskInput input, std::size_t unique_id) const override;

protected:
  CheckInputFn fn_;

  /**
   * @brief Checks that the TaskInput is in the correct format.
   * @param input TaskInput to be checked
   * @return True if in the correct format
   */
  static bool checkTaskInput(const TaskInput& input);
};

class CheckInputTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<CheckInputTaskInfo>;
  using ConstPtr = std::shared_ptr<const CheckInputTaskInfo>;

  CheckInputTaskInfo(std::size_t unique_id, std::string name = "Check TaksInput");
};
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_CHECK_INPUT_TASK_GENERATOR_H

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
#include <boost/serialization/access.hpp>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
using CheckInputFn = std::function<bool(const TaskInput&)>;

class CheckInputTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<CheckInputTaskGenerator>;

  CheckInputTaskGenerator(std::string name = profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE);
  ~CheckInputTaskGenerator() override = default;
  CheckInputTaskGenerator(const CheckInputTaskGenerator&) = delete;
  CheckInputTaskGenerator& operator=(const CheckInputTaskGenerator&) = delete;
  CheckInputTaskGenerator(CheckInputTaskGenerator&&) = delete;
  CheckInputTaskGenerator& operator=(CheckInputTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;
};

class CheckInputTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<CheckInputTaskInfo>;
  using ConstPtr = std::shared_ptr<const CheckInputTaskInfo>;

  CheckInputTaskInfo() = default;
  CheckInputTaskInfo(std::size_t unique_id, std::string name = profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

  bool operator==(const CheckInputTaskInfo& rhs) const;
  bool operator!=(const CheckInputTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::CheckInputTaskInfo, "CheckInputTaskInfo")

#endif  // TESSERACT_PROCESS_MANAGERS_CHECK_INPUT_TASK_GENERATOR_H

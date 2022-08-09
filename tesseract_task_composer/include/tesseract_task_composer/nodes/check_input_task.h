/**
 * @file check_input_task.h
 * @brief Task for checking input data structure
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
#ifndef TESSERACT_TASK_COMPOSER_CHECK_INPUT_TASK_H
#define TESSERACT_TASK_COMPOSER_CHECK_INPUT_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>

namespace tesseract_planning
{
class CheckInputTask : public TaskComposerNode
{
public:
  using Ptr = std::shared_ptr<CheckInputTask>;
  using ConstPtr = std::shared_ptr<const CheckInputTask>;
  using UPtr = std::unique_ptr<CheckInputTask>;
  using ConstUPtr = std::unique_ptr<const CheckInputTask>;

  CheckInputTask() = default;  // Required for serialization
  CheckInputTask(std::string input_key, std::string name = profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE);
  CheckInputTask(std::vector<std::string> input_keys, std::string name = profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE);
  ~CheckInputTask() override = default;
  CheckInputTask(const CheckInputTask&) = delete;
  CheckInputTask& operator=(const CheckInputTask&) = delete;
  CheckInputTask(CheckInputTask&&) = delete;
  CheckInputTask& operator=(CheckInputTask&&) = delete;

  int run(TaskComposerInput& input) const override final;

  bool operator==(const CheckInputTask& rhs) const;
  bool operator!=(const CheckInputTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::vector<std::string> input_keys_;
};

class CheckInputTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<CheckInputTaskInfo>;
  using ConstPtr = std::shared_ptr<const CheckInputTaskInfo>;
  using UPtr = std::unique_ptr<CheckInputTaskInfo>;
  using ConstUPtr = std::unique_ptr<const CheckInputTaskInfo>;

  CheckInputTaskInfo() = default;
  CheckInputTaskInfo(boost::uuids::uuid uuid, std::string name = profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE);

  TaskComposerNodeInfo::UPtr clone() const override;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::CheckInputTask, "CheckInputTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::CheckInputTaskInfo, "CheckInputTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_CHECK_INPUT_TASK_H

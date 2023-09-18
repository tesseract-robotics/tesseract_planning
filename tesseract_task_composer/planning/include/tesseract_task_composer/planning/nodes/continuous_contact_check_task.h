/**
 * @file continuous_contact_check_task.h
 * @brief Continuous Collision check trajectory task
 *
 * @author Levi Armstrong
 * @date August 10. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_CONTINUOUS_CONTACT_CHECK_TASK_H
#define TESSERACT_TASK_COMPOSER_CONTINUOUS_CONTACT_CHECK_TASK_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class ContinuousContactCheckTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<ContinuousContactCheckTask>;
  using ConstPtr = std::shared_ptr<const ContinuousContactCheckTask>;
  using UPtr = std::unique_ptr<ContinuousContactCheckTask>;
  using ConstUPtr = std::unique_ptr<const ContinuousContactCheckTask>;

  ContinuousContactCheckTask();
  explicit ContinuousContactCheckTask(std::string name, std::string input_key, bool conditional = true);
  explicit ContinuousContactCheckTask(std::string name,
                                      const YAML::Node& config,
                                      const TaskComposerPluginFactory& plugin_factory);

  ~ContinuousContactCheckTask() override = default;
  ContinuousContactCheckTask(const ContinuousContactCheckTask&) = delete;
  ContinuousContactCheckTask& operator=(const ContinuousContactCheckTask&) = delete;
  ContinuousContactCheckTask(ContinuousContactCheckTask&&) = delete;
  ContinuousContactCheckTask& operator=(ContinuousContactCheckTask&&) = delete;

  bool operator==(const ContinuousContactCheckTask& rhs) const;
  bool operator!=(const ContinuousContactCheckTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

class ContinuousContactCheckTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<ContinuousContactCheckTaskInfo>;
  using ConstPtr = std::shared_ptr<const ContinuousContactCheckTaskInfo>;
  using UPtr = std::unique_ptr<ContinuousContactCheckTaskInfo>;
  using ConstUPtr = std::unique_ptr<const ContinuousContactCheckTaskInfo>;

  ContinuousContactCheckTaskInfo() = default;
  ContinuousContactCheckTaskInfo(const ContinuousContactCheckTask& task);

  tesseract_environment::Environment::ConstPtr env;
  std::vector<tesseract_collision::ContactResultMap> contact_results;

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const ContinuousContactCheckTaskInfo& rhs) const;
  bool operator!=(const ContinuousContactCheckTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ContinuousContactCheckTask, "ContinuousContactCheckTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ContinuousContactCheckTaskInfo, "ContinuousContactCheckTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_CONTINUOUS_CONTACT_CHECK_TASK_H

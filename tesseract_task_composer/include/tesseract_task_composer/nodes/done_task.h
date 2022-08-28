/**
 * @file done_task.h
 *
 * @author Levi Armstrong
 * @date August 5, 2022
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
#ifndef TESSERACT_TASK_COMPOSER_DONE_TASK_H
#define TESSERACT_TASK_COMPOSER_DONE_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>

namespace tesseract_planning
{
class DoneTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<DoneTask>;
  using ConstPtr = std::shared_ptr<const DoneTask>;
  using UPtr = std::unique_ptr<DoneTask>;
  using ConstUPtr = std::unique_ptr<const DoneTask>;

  DoneTask(bool is_conditional = false, std::string name = "DoneTask");
  ~DoneTask() = default;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const DoneTask& rhs) const;
  bool operator!=(const DoneTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class DoneTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<DoneTaskInfo>;
  using ConstPtr = std::shared_ptr<const DoneTaskInfo>;
  using UPtr = std::unique_ptr<DoneTaskInfo>;
  using ConstUPtr = std::unique_ptr<const DoneTaskInfo>;

  DoneTaskInfo() = default;
  DoneTaskInfo(boost::uuids::uuid uuid, std::string name);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const DoneTaskInfo& rhs) const;
  bool operator!=(const DoneTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::DoneTask, "DoneTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::DoneTaskInfo, "DoneTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_DONE_TASK_H

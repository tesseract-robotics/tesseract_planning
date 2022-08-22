/**
 * @file update_end_state_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_UPDATE_END_STATE_TASK_H
#define TESSERACT_TASK_COMPOSER_UPDATE_END_STATE_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>

namespace tesseract_planning
{
class UpdateEndStateTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<UpdateEndStateTask>;
  using ConstPtr = std::shared_ptr<const UpdateEndStateTask>;
  using UPtr = std::unique_ptr<UpdateEndStateTask>;
  using ConstUPtr = std::unique_ptr<const UpdateEndStateTask>;

  UpdateEndStateTask() = default;
  /** @brief The input_key is the uuid string */
  UpdateEndStateTask(std::string input_next_key,
                     std::string output_key,
                     bool is_conditional = false,
                     std::string name = "UpdateEndStateTask");

  UpdateEndStateTask(std::string input_key,
                     std::string input_next_key,
                     std::string output_key,
                     bool is_conditional = false,
                     std::string name = "UpdateEndStateTask");
  ~UpdateEndStateTask() override = default;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor = std::nullopt) const override;

  bool operator==(const UpdateEndStateTask& rhs) const;
  bool operator!=(const UpdateEndStateTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string input_key_;
  std::string input_next_key_;
  std::string output_key_;
};

class UpdateEndStateTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<UpdateEndStateTaskInfo>;
  using ConstPtr = std::shared_ptr<const UpdateEndStateTaskInfo>;
  using UPtr = std::unique_ptr<UpdateEndStateTaskInfo>;
  using ConstUPtr = std::unique_ptr<const UpdateEndStateTaskInfo>;

  UpdateEndStateTaskInfo() = default;
  UpdateEndStateTaskInfo(boost::uuids::uuid uuid, std::string name);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const UpdateEndStateTaskInfo& rhs) const;
  bool operator!=(const UpdateEndStateTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::UpdateEndStateTask, "UpdateEndStateTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::UpdateEndStateTaskInfo, "UpdateEndStateTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_UPDATE_END_STATE_TASK_H

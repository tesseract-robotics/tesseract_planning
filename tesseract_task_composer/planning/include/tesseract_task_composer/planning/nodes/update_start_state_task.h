/**
 * @file update_start_state_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_UPDATE_START_STATE_TASK_H
#define TESSERACT_TASK_COMPOSER_UPDATE_START_STATE_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class UpdateStartStateTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<UpdateStartStateTask>;
  using ConstPtr = std::shared_ptr<const UpdateStartStateTask>;
  using UPtr = std::unique_ptr<UpdateStartStateTask>;
  using ConstUPtr = std::unique_ptr<const UpdateStartStateTask>;

  UpdateStartStateTask() = default;
  /** @brief The input_key is the uuid string */
  explicit UpdateStartStateTask(std::string name, std::string input_prev_key, std::string output_key, bool conditional);

  explicit UpdateStartStateTask(std::string name,
                                std::string input_key,
                                std::string input_prev_key,
                                std::string output_key,
                                bool conditional);
  ~UpdateStartStateTask() override = default;

  bool operator==(const UpdateStartStateTask& rhs) const;
  bool operator!=(const UpdateStartStateTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::UpdateStartStateTask, "UpdateStartStateTask")

#endif  // TESSERACT_TASK_COMPOSER_UPDATE_START_STATE_TASK_H

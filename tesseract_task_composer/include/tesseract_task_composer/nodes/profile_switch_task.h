/**
 * @file profile_switch_task.h
 * @brief Task that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_TASK_H
#define TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>

namespace tesseract_planning
{
/**
 * @brief This task simply returns a value specified in the composite profile. This can be used to switch execution
 * based on the profile
 */
class ProfileSwitchTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<ProfileSwitchTask>;
  using ConstPtr = std::shared_ptr<const ProfileSwitchTask>;
  using UPtr = std::unique_ptr<ProfileSwitchTask>;
  using ConstUPtr = std::unique_ptr<const ProfileSwitchTask>;

  ProfileSwitchTask() = default;  // Required for serialization
  ProfileSwitchTask(std::string input_key,
                    bool is_conditional = true,
                    std::string name = profile_ns::PROFILE_SWITCH_DEFAULT_NAMESPACE);
  ~ProfileSwitchTask() override = default;
  ProfileSwitchTask(const ProfileSwitchTask&) = delete;
  ProfileSwitchTask& operator=(const ProfileSwitchTask&) = delete;
  ProfileSwitchTask(ProfileSwitchTask&&) = delete;
  ProfileSwitchTask& operator=(ProfileSwitchTask&&) = delete;

  int run(TaskComposerInput& input) const override final;

  bool operator==(const ProfileSwitchTask& rhs) const;
  bool operator!=(const ProfileSwitchTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string input_key_;
};

class ProfileSwitchTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<ProfileSwitchTaskInfo>;
  using ConstPtr = std::shared_ptr<const ProfileSwitchTaskInfo>;
  using UPtr = std::unique_ptr<ProfileSwitchTaskInfo>;
  using ConstUPtr = std::unique_ptr<const ProfileSwitchTaskInfo>;

  ProfileSwitchTaskInfo() = default;
  ProfileSwitchTaskInfo(boost::uuids::uuid uuid, std::string name = profile_ns::PROFILE_SWITCH_DEFAULT_NAMESPACE);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const ProfileSwitchTaskInfo& rhs) const;
  bool operator!=(const ProfileSwitchTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ProfileSwitchTask, "ProfileSwitchTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ProfileSwitchTaskInfo, "ProfileSwitchTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_TASK_H

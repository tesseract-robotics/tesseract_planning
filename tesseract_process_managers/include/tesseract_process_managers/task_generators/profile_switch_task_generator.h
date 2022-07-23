/**
 * @file profile_switch_task_generator.h
 * @brief Process generator that returns a value based on the profile
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
/**
 * @brief This generator simply returns a value specified in the composite profile. This can be used to switch execution
 * based on the profile
 */
class ProfileSwitchTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<ProfileSwitchTaskGenerator>;

  ProfileSwitchTaskGenerator(std::string name = profile_ns::PROFILE_SWITCH_DEFAULT_NAMESPACE);

  ~ProfileSwitchTaskGenerator() override = default;
  ProfileSwitchTaskGenerator(const ProfileSwitchTaskGenerator&) = delete;
  ProfileSwitchTaskGenerator& operator=(const ProfileSwitchTaskGenerator&) = delete;
  ProfileSwitchTaskGenerator(ProfileSwitchTaskGenerator&&) = delete;
  ProfileSwitchTaskGenerator& operator=(ProfileSwitchTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;
};

class ProfileSwitchTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<ProfileSwitchTaskInfo>;
  using ConstPtr = std::shared_ptr<const ProfileSwitchTaskInfo>;

  ProfileSwitchTaskInfo() = default;
  ProfileSwitchTaskInfo(std::size_t unique_id, std::string name = profile_ns::PROFILE_SWITCH_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ProfileSwitchTaskInfo, "ProfileSwitchTaskInfo")
#endif  // TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_TASK_GENERATOR_H

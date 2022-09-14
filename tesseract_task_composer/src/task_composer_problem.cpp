/**
 * @file task_composer_problem.cpp
 * @brief A task composer server problem
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/shared_ptr.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_problem.h>

namespace tesseract_planning
{
TaskComposerProblem::TaskComposerProblem(TaskComposerDataStorage input_data, std::string name)
  : name(std::move(name)), input_data(std::move(input_data))
{
}

TaskComposerProblem::TaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                         tesseract_common::ManipulatorInfo manip_info,
                                         TaskComposerDataStorage input_data,
                                         std::string name)
  : name(std::move(name)), env(std::move(env)), manip_info(std::move(manip_info)), input_data(std::move(input_data))
{
}

TaskComposerProblem::TaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                         tesseract_common::ManipulatorInfo manip_info,
                                         ProfileRemapping move_profile_remapping,
                                         ProfileRemapping composite_profile_remapping,
                                         TaskComposerDataStorage input_data,
                                         std::string name)
  : name(std::move(name))
  , env(std::move(env))
  , manip_info(std::move(manip_info))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
  , input_data(std::move(input_data))
{
}

TaskComposerProblem::TaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                         ProfileRemapping move_profile_remapping,
                                         ProfileRemapping composite_profile_remapping,
                                         TaskComposerDataStorage input_data,
                                         std::string name)
  : name(std::move(name))
  , env(std::move(env))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
  , input_data(std::move(input_data))
{
}

TaskComposerProblem::TaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                         TaskComposerDataStorage input_data,
                                         std::string name)
  : name(std::move(name)), env(std::move(env)), input_data(std::move(input_data))
{
}

bool TaskComposerProblem::operator==(const TaskComposerProblem& rhs) const
{
  bool equal = true;
  equal &= name == rhs.name;
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  equal &= manip_info == rhs.manip_info;
  equal &= move_profile_remapping == rhs.move_profile_remapping;
  equal &= composite_profile_remapping == rhs.composite_profile_remapping;
  equal &= input_data == rhs.input_data;
  return equal;
}

bool TaskComposerProblem::operator!=(const TaskComposerProblem& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerProblem::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name);
  ar& boost::serialization::make_nvp("environment", env);
  ar& boost::serialization::make_nvp("manip_info", manip_info);
  ar& boost::serialization::make_nvp("move_profile_remapping", move_profile_remapping);
  ar& boost::serialization::make_nvp("composite_profile_remapping", composite_profile_remapping);
  ar& boost::serialization::make_nvp("input_data", input_data);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerProblem)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerProblem)

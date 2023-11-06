/**
 * @file planning task_composer_problem.h
 * @brief A task composer server planning problem
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

#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

namespace tesseract_planning
{
PlanningTaskComposerProblem::PlanningTaskComposerProblem(std::string name) : TaskComposerProblem(std::move(name)) {}

PlanningTaskComposerProblem::PlanningTaskComposerProblem(ProfileDictionary::ConstPtr profiles, std::string name)
  : TaskComposerProblem(std::move(name)), profiles(std::move(profiles))
{
}

PlanningTaskComposerProblem::PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                                         tesseract_common::ManipulatorInfo manip_info,
                                                         ProfileDictionary::ConstPtr profiles,
                                                         std::string name)
  : TaskComposerProblem(std::move(name))
  , env(std::move(env))
  , manip_info(std::move(manip_info))
  , profiles(std::move(profiles))
{
}

PlanningTaskComposerProblem::PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                                         tesseract_common::ManipulatorInfo manip_info,
                                                         ProfileRemapping move_profile_remapping,
                                                         ProfileRemapping composite_profile_remapping,
                                                         ProfileDictionary::ConstPtr profiles,
                                                         std::string name)
  : TaskComposerProblem(std::move(name))
  , env(std::move(env))
  , manip_info(std::move(manip_info))
  , profiles(std::move(profiles))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
{
}

PlanningTaskComposerProblem::PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                                         ProfileRemapping move_profile_remapping,
                                                         ProfileRemapping composite_profile_remapping,
                                                         ProfileDictionary::ConstPtr profiles,
                                                         std::string name)
  : TaskComposerProblem(std::move(name))
  , env(std::move(env))
  , profiles(std::move(profiles))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
{
}

PlanningTaskComposerProblem::PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                                                         ProfileDictionary::ConstPtr profiles,
                                                         std::string name)
  : TaskComposerProblem(std::move(name)), env(std::move(env)), profiles(std::move(profiles))
{
}

TaskComposerProblem::UPtr PlanningTaskComposerProblem::clone() const
{
  return std::make_unique<PlanningTaskComposerProblem>(*this);
}

bool PlanningTaskComposerProblem::operator==(const PlanningTaskComposerProblem& rhs) const
{
  bool equal = true;
  equal &= TaskComposerProblem::operator==(rhs);
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  equal &= manip_info == rhs.manip_info;
  //  equal &= tesseract_common::pointersEqual(profiles, rhs.profiles);
  equal &= move_profile_remapping == rhs.move_profile_remapping;
  equal &= composite_profile_remapping == rhs.composite_profile_remapping;
  return equal;
}

bool PlanningTaskComposerProblem::operator!=(const PlanningTaskComposerProblem& rhs) const { return !operator==(rhs); }

template <class Archive>
void PlanningTaskComposerProblem::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerProblem);
  ar& boost::serialization::make_nvp("environment", env);
  ar& boost::serialization::make_nvp("manip_info", manip_info);
  /** @todo Fix after profiles are serializable */
  //  ar& boost::serialization::make_nvp("profiles", profiles);
  ar& boost::serialization::make_nvp("move_profile_remapping", move_profile_remapping);
  ar& boost::serialization::make_nvp("composite_profile_remapping", composite_profile_remapping);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::PlanningTaskComposerProblem)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::PlanningTaskComposerProblem)

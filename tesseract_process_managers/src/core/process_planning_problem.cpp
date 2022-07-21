/**
 * @file process_planning_results.cpp
 * @brief A process planning results
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
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_planning_problem.h>

namespace tesseract_planning
{
ProcessPlanningProblem::ProcessPlanningProblem(const ProcessPlanningProblem& other) { copy(other); }

ProcessPlanningProblem& ProcessPlanningProblem::operator=(const ProcessPlanningProblem& other)
{
  if (&other != this)
    copy(other);
  return *this;
}

void ProcessPlanningProblem::copy(const ProcessPlanningProblem& other)
{
  name = other.name;
  env = other.env;
  if (other.input != nullptr)
    input = std::make_unique<InstructionPoly>(*other.input);

  if (other.results != nullptr)
    results = std::make_unique<InstructionPoly>(*other.results);

  if (other.global_manip_info != nullptr)
    global_manip_info = std::make_unique<tesseract_common::ManipulatorInfo>(*other.global_manip_info);

  if (other.plan_profile_remapping != nullptr)
    plan_profile_remapping = std::make_unique<PlannerProfileRemapping>(*other.plan_profile_remapping);

  if (other.composite_profile_remapping != nullptr)
    composite_profile_remapping = std::make_unique<PlannerProfileRemapping>(*other.composite_profile_remapping);
}

bool ProcessPlanningProblem::operator==(const tesseract_planning::ProcessPlanningProblem& rhs) const
{
  bool equal = true;
  equal &= (name == rhs.name);
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  equal &= (input && rhs.input && *input == *rhs.input) || (!input && !rhs.input);
  equal &= (results && rhs.results && *results == *rhs.results) || (!results && !rhs.results);
  equal &= (global_manip_info && rhs.global_manip_info && *global_manip_info == *rhs.global_manip_info) ||
           (!global_manip_info && !rhs.global_manip_info);
  equal &= (plan_profile_remapping && rhs.plan_profile_remapping &&
            *plan_profile_remapping == *rhs.plan_profile_remapping) ||
           (!plan_profile_remapping && !rhs.plan_profile_remapping);
  equal &= (composite_profile_remapping && rhs.composite_profile_remapping &&
            *composite_profile_remapping == *rhs.composite_profile_remapping) ||
           (!composite_profile_remapping && !rhs.composite_profile_remapping);

  return equal;
}

bool ProcessPlanningProblem::operator!=(const tesseract_planning::ProcessPlanningProblem& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ProcessPlanningProblem::serialize(Archive& ar, const unsigned int version)
{
  if (version >= 1)
    ar& BOOST_SERIALIZATION_NVP(name);

  ar& BOOST_SERIALIZATION_NVP(env);
  ar& BOOST_SERIALIZATION_NVP(input);
  ar& BOOST_SERIALIZATION_NVP(results);
  ar& BOOST_SERIALIZATION_NVP(global_manip_info);
  ar& BOOST_SERIALIZATION_NVP(plan_profile_remapping);
  ar& BOOST_SERIALIZATION_NVP(composite_profile_remapping);
  // This is not currently serializable
  //  ar& BOOST_SERIALIZATION_NVP(taskflow_container);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProcessPlanningProblem)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProcessPlanningProblem)

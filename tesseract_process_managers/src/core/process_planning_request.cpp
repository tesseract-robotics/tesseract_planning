/**
 * @file process_planning_request.cpp
 * @brief A process planning request and default planner names
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 15, 2022
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_common/utils.h>

bool tesseract_planning::ProcessPlanningRequest::operator==(const tesseract_planning::ProcessPlanningRequest& rhs) const
{
  using namespace tesseract_common;
  using namespace tesseract_environment;
  bool equal = true;
  equal &= executor_name == rhs.executor_name;
  equal &= name == rhs.name;
  equal &= instructions == rhs.instructions;
  equal &= seed == rhs.seed;
  equal &= env_state == rhs.env_state;
  equal &= tesseract_common::isIdentical<Command::ConstPtr>(commands, rhs.commands, true, pointersEqual<const Command>);
  equal &= profile == rhs.profile;
  equal &= save_io == rhs.save_io;
  equal &= plan_profile_remapping == rhs.plan_profile_remapping;
  equal &= composite_profile_remapping == rhs.composite_profile_remapping;
  return equal;
}

bool tesseract_planning::ProcessPlanningRequest::operator!=(const tesseract_planning::ProcessPlanningRequest& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void tesseract_planning::ProcessPlanningRequest::serialize(Archive& ar, const unsigned int version)
{
  if (version >= 1)
    ar& BOOST_SERIALIZATION_NVP(executor_name);

  ar& BOOST_SERIALIZATION_NVP(name);
  ar& BOOST_SERIALIZATION_NVP(instructions);
  ar& BOOST_SERIALIZATION_NVP(seed);
  ar& BOOST_SERIALIZATION_NVP(env_state);
  ar& BOOST_SERIALIZATION_NVP(commands);
  ar& BOOST_SERIALIZATION_NVP(profile);
  ar& BOOST_SERIALIZATION_NVP(save_io);
  ar& BOOST_SERIALIZATION_NVP(plan_profile_remapping);
  ar& BOOST_SERIALIZATION_NVP(composite_profile_remapping);
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProcessPlanningRequest)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProcessPlanningRequest)

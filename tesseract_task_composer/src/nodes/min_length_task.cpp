/**
 * @file min_length_task.cpp
 * @brief Task for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/profiles/seed_min_length_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
MinLengthTask::MinLengthTask(std::string input_key, std::string output_key, bool is_conditional, std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

int MinLengthTask::run(TaskComposerInput& input, OptionalTaskComposerExecutor /*executor*/) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<MinLengthTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  // Check that inputs are valid
  auto input_data_poly = input.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    CONSOLE_BRIDGE_logError("Input seed to SeedMinLengthTask must be a composite instruction");
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  long cnt = ci.getMoveInstructionCount();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<SeedMinLengthProfile>(name_, profile, *input.profiles, std::make_shared<SeedMinLengthProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.profile_overrides);

  if (cnt < cur_composite_profile->min_length)
  {
    auto subdivisions =
        static_cast<int>(std::ceil(static_cast<double>(cur_composite_profile->min_length) / static_cast<double>(cnt))) +
        1;

    // Fill out request and response
    PlannerRequest request;
    request.instructions = ci;
    request.env_state = input.env->getState();
    request.env = input.env;

    // Set up planner
    SimpleMotionPlanner planner;

    auto profile = std::make_shared<SimplePlannerFixedSizePlanProfile>(subdivisions, subdivisions);

    // Create profile dictionary
    auto profiles = std::make_shared<ProfileDictionary>();
    profiles->addProfile<SimplePlannerPlanProfile>(planner.getName(), ci.getProfile(), profile);
    auto flat = ci.flatten(&moveFilter);
    for (const auto& i : flat)
      profiles->addProfile<SimplePlannerPlanProfile>(
          planner.getName(), i.get().as<MoveInstructionPoly>().getProfile(), profile);

    // Assign profile dictionary
    request.profiles = profiles;

    // Solve
    PlannerResponse response = planner.solve(request);

    if (!response.successful)
    {
      CONSOLE_BRIDGE_logError("SeedMinLengthTask, failed to subdivid!");
      //    saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      input.addTaskInfo(std::move(info));
      return 0;
    }

    input.data_storage->setData(output_keys_[0], response.results);
  }
  else
  {
    input.data_storage->setData(output_keys_[0], ci);
  }

  CONSOLE_BRIDGE_logDebug("Seed Min Length Task Succeeded!");
  info->return_value = 1;
  //  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

TaskComposerNode::UPtr MinLengthTask::clone() const
{
  return std::make_unique<MinLengthTask>(input_keys_[0], output_keys_[0], is_conditional_, name_);
}

bool MinLengthTask::operator==(const MinLengthTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool MinLengthTask::operator!=(const MinLengthTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void MinLengthTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

MinLengthTaskInfo::MinLengthTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr MinLengthTaskInfo::clone() const { return std::make_unique<MinLengthTaskInfo>(*this); }

bool MinLengthTaskInfo::operator==(const MinLengthTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool MinLengthTaskInfo::operator!=(const MinLengthTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void MinLengthTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MinLengthTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MinLengthTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MinLengthTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MinLengthTaskInfo)

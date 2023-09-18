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

#include <tesseract_task_composer/planning/nodes/min_length_task.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
MinLengthTask::MinLengthTask() : TaskComposerTask("MinLengthTask", false) {}
MinLengthTask::MinLengthTask(std::string name, std::string input_key, std::string output_key, bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

MinLengthTask::MinLengthTask(std::string name,
                             const YAML::Node& config,
                             const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("MinLengthTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("MinLengthTask, config 'inputs' entry currently only supports one input key");

  if (output_keys_.empty())
    throw std::runtime_error("MinLengthTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("MinLengthTask, config 'outputs' entry currently only supports one output key");
}

TaskComposerNodeInfo::UPtr MinLengthTask::runImpl(TaskComposerContext& context,
                                                  OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  // Check that inputs are valid
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input seed to MinLengthTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  long cnt = ci.getMoveInstructionCount();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, problem.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<MinLengthProfile>(name_, profile, *problem.profiles, std::make_shared<MinLengthProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  if (cnt < cur_composite_profile->min_length)
  {
    auto subdivisions = static_cast<int>(
        std::ceil(static_cast<double>(cur_composite_profile->min_length) / static_cast<double>(cnt - 1)));

    // Fill out request and response
    PlannerRequest request;
    request.instructions = ci;
    request.env_state = problem.env->getState();
    request.env = problem.env;

    // Set up planner
    SimpleMotionPlanner planner(name_);

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
      info->message = "MinLengthTask, failed to subdivid!";
      CONSOLE_BRIDGE_logError("%s", info->message.c_str());
      return info;
    }

    context.data_storage->setData(output_keys_[0], response.results);
  }
  else
  {
    context.data_storage->setData(output_keys_[0], ci);
  }

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("Seed Min Length Task Succeeded!");
  return info;
}

bool MinLengthTask::operator==(const MinLengthTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool MinLengthTask::operator!=(const MinLengthTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void MinLengthTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MinLengthTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MinLengthTask)

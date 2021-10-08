/**
 * @file iterative_spline_parameterization_task_generator.cpp
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>

namespace tesseract_planning
{
IterativeSplineParameterizationProfile::IterativeSplineParameterizationProfile(double max_velocity_scaling_factor,
                                                                               double max_acceleration_scaling_factor)
  : max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

IterativeSplineParameterizationTaskGenerator::IterativeSplineParameterizationTaskGenerator(bool add_points,
                                                                                           std::string name)
  : TaskGenerator(std::move(name)), solver_(add_points)
{
  // Register default profile
  composite_profiles["DEFAULT"] = std::make_shared<IterativeSplineParameterizationProfile>();
}

int IterativeSplineParameterizationTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<IterativeSplineParameterizationTaskInfo>(unique_id, name_);
  info->return_value = 0;
  input.addTaskInfo(info);
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  Instruction* input_results = input.getResults();
  if (!isCompositeInstruction(*input_results))
  {
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 0;
  }

  auto& ci = input_results->as<CompositeInstruction>();
  const ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = input.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(profile, name_, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<IterativeSplineParameterizationProfile>(
      profile, composite_profiles, std::make_shared<IterativeSplineParameterizationProfile>());
  cur_composite_profile = applyProfileOverrides(name_, cur_composite_profile, ci.profile_overrides);

  // Create data structures for checking for plan profile overrides
  auto flattened = flatten(ci, moveFilter);
  if (flattened.empty())
  {
    CONSOLE_BRIDGE_logWarn("Iterative spline time parameterization found no MoveInstructions to process");
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 1;
  }

  Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                             cur_composite_profile->max_velocity_scaling_factor;
  Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                 cur_composite_profile->max_acceleration_scaling_factor;

  // Loop over all PlanInstructions
  for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
  {
    const auto& mi = flattened[static_cast<std::size_t>(idx)].get().as<MoveInstruction>();
    std::string plan_profile = mi.getProfile();

    // Check for remapping of the plan profile
    plan_profile = getProfileString(profile, name_, input.plan_profile_remapping);
    auto cur_move_profile =
        getProfile<IterativeSplineParameterizationProfile>(plan_profile, composite_profiles, nullptr);
    cur_move_profile = applyProfileOverrides(name_, cur_move_profile, mi.profile_overrides);

    // If there is a move profile associated with it, override the parameters
    if (cur_move_profile)
    {
      velocity_scaling_factors[idx] = cur_move_profile->max_velocity_scaling_factor;
      acceleration_scaling_factors[idx] = cur_move_profile->max_acceleration_scaling_factor;
    }
  }

  // Solve using parameters
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(ci);
  if (!solver_.compute(*trajectory,
                       limits.velocity_limits,
                       limits.acceleration_limits,
                       velocity_scaling_factors,
                       acceleration_scaling_factors))
  {
    CONSOLE_BRIDGE_logInform("Failed to perform iterative spline time parameterization for process input: %s!",
                             input_results->getDescription().c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  return 1;
}

void IterativeSplineParameterizationTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

IterativeSplineParameterizationTaskInfo::IterativeSplineParameterizationTaskInfo(std::size_t unique_id,
                                                                                 std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning

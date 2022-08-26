/**
 * @file time_optimal_trajectory_generation_task_generator.h
 * @brief Perform TOTG
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date January 22, 2021
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
#include <tesseract_process_managers/task_generators/time_optimal_parameterization_task_generator.h>
#include <tesseract_process_managers/task_profiles/time_optimal_parameterization_profile.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_time_parameterization/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/utils.h>

namespace tesseract_planning
{
TimeOptimalParameterizationTaskGenerator::TimeOptimalParameterizationTaskGenerator(std::string name)
  : TaskGenerator(std::move(name))
{
}

int TimeOptimalParameterizationTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<TimeOptimalTrajectoryGenerationTaskInfo>(unique_id, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  InstructionPoly* input_results = input.getResults();
  if (!input_results->isCompositeInstruction())
  {
    CONSOLE_BRIDGE_logError("Input results to TOTG must be a composite instruction");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  auto& ci = input_results->as<CompositeInstruction>();
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = input.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<TimeOptimalParameterizationProfile>(
      name_, profile, *input.profiles, std::make_shared<TimeOptimalParameterizationProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    CONSOLE_BRIDGE_logWarn("TOTG found no MoveInstructions to process");
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  double velocity_scaling_factor = cur_composite_profile->max_velocity_scaling_factor;
  double acceleration_scaling_factor = cur_composite_profile->max_acceleration_scaling_factor;

  // Loop over all sub-composites
  bool use_move_profile = false;
  std::vector<double> scaling_factors(ci.size(), velocity_scaling_factor);
  for (std::size_t idx = 0; idx < ci.size(); idx++)
  {
    const auto& mi = ci.at(idx).as<CompositeInstruction>();
    profile = mi.getProfile();

    // Check for remapping of the plan profile
    std::string remap = getProfileString(name_, profile, input.plan_profile_remapping);
    auto cur_move_profile = getProfile<TimeOptimalParameterizationProfile>(
        name_, remap, *input.profiles, std::make_shared<TimeOptimalParameterizationProfile>());
    cur_move_profile = applyProfileOverrides(name_, remap, cur_move_profile, mi.getProfileOverrides());

    // If there is a move profile associated with it, override the parameters
    if (cur_move_profile)
    {
      use_move_profile = true;
      scaling_factors[idx] = cur_move_profile->max_velocity_scaling_factor;
    }
  }

  if (use_move_profile)
  {
    // Set these to 1 so we can do the scaling after the fact
    velocity_scaling_factor = 1;
    acceleration_scaling_factor = 1;
  }

  // Solve using parameters
  TimeOptimalTrajectoryGeneration solver(cur_composite_profile->path_tolerance,
                                         cur_composite_profile->resample_dt,
                                         cur_composite_profile->min_angle_change);

  // Copy the Composite before passing in because it will get flattened and resampled
  CompositeInstruction resampled(ci);
  if (!solver.computeTimeStamps(resampled,
                                limits.velocity_limits,
                                limits.acceleration_limits,
                                velocity_scaling_factor,
                                acceleration_scaling_factor))
  {
    CONSOLE_BRIDGE_logInform("Failed to perform TOTG for process input: %s!", input_results->getDescription().c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Unflatten
  if (cur_composite_profile->unflatten)
  {
    CompositeInstruction unflattened = unflatten(resampled, ci, cur_composite_profile->unflatten_tolerance);

    // Rescale
    if (use_move_profile)
    {
      RescaleTimings(unflattened, scaling_factors);
    }

    ci.setStartInstruction(unflattened.getStartInstruction());
    for (std::size_t idx = 0; idx < ci.size(); idx++)
      ci[idx] = unflattened[idx];
  }
  else
  {
    if (use_move_profile)
      CONSOLE_BRIDGE_logWarn("TOTG Move Profile specified but unflatten is not set in the composite profile. Move "
                             "Profile will be ignored");

    ci.setStartInstruction(resampled.getStartInstruction());
    for (std::size_t idx = 0; idx < ci.size(); idx++)
      ci[idx] = resampled[idx];
  }

  CONSOLE_BRIDGE_logDebug("TOTG succeeded");
  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

void TimeOptimalParameterizationTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

CompositeInstruction TimeOptimalParameterizationTaskGenerator::unflatten(const CompositeInstruction& flattened_input,
                                                                         const CompositeInstruction& pattern,
                                                                         double tolerance)
{
  CompositeInstruction unflattened(pattern);
  unflattened.setStartInstruction(flattened_input.getStartInstruction());
  for (auto& instr : unflattened)
    instr.as<CompositeInstruction>().clear();

  Eigen::VectorXd last_pt_in_input =
      getJointPosition(pattern.at(0).as<CompositeInstruction>().back().as<MoveInstructionPoly>().getWaypoint());

  double error = 0;
  double prev_error = 1;
  bool hit_tolerance = false;
  bool error_increasing = false;

  // Loop over the flattened composite adding the instructions to the appropriate subcomposite
  for (std::size_t resample_idx = 0, original_idx = 0; resample_idx < flattened_input.size(); resample_idx++)
  {
    // If all joints are within the tolerance, then this point hopefully corresponds
    if (flattened_input.at(resample_idx).isMoveInstruction())
    {
      // Get the current position to see if we should increment original_idx
      const Eigen::VectorXd& current_pt =
          getJointPosition(flattened_input.at(resample_idx).as<MoveInstructionPoly>().getWaypoint());
      error = (last_pt_in_input - current_pt).cwiseAbs().maxCoeff();

      // Check if we've hit the tolerance and if the error is still decreasing
      if (error < tolerance)
      {
        hit_tolerance = true;
      }
      if (prev_error < error)
      {
        error_increasing = true;
      }
      prev_error = error;

      // Wait until the tolerance has been satisfied and the error isn't decreasing anymore before switching composites
      if (hit_tolerance && error_increasing)
      {
        if (original_idx < pattern.size() - 1)  // Keep from incrementing too far at the end of the last composite
          original_idx++;
        last_pt_in_input = getJointPosition(
            pattern.at(original_idx).as<CompositeInstruction>().back().as<MoveInstructionPoly>().getWaypoint());

        hit_tolerance = false;
        error_increasing = false;
      }
    }

    // Add flattened point to the subcomposite
    unflattened[original_idx].as<CompositeInstruction>().appendInstruction(flattened_input.at(resample_idx));

    // Correct the meta information, taking information from the last element of each composite in the original
    if (unflattened[original_idx].as<CompositeInstruction>().back().isMoveInstruction())
    {
      const auto& pattern_instr = pattern.at(original_idx).as<CompositeInstruction>().back().as<MoveInstructionPoly>();
      unflattened[original_idx].as<CompositeInstruction>().back().as<MoveInstructionPoly>().setMoveType(
          static_cast<MoveInstructionType>(pattern_instr.getMoveType()));
      unflattened[original_idx].as<CompositeInstruction>().back().as<MoveInstructionPoly>().setProfile(
          pattern_instr.getProfile());
    }
  }
  return unflattened;
}

TimeOptimalTrajectoryGenerationTaskInfo::TimeOptimalTrajectoryGenerationTaskInfo(std::size_t unique_id,
                                                                                 std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr TimeOptimalTrajectoryGenerationTaskInfo::clone() const
{
  return std::make_unique<TimeOptimalTrajectoryGenerationTaskInfo>(*this);
}

bool TimeOptimalTrajectoryGenerationTaskInfo::operator==(const TimeOptimalTrajectoryGenerationTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool TimeOptimalTrajectoryGenerationTaskInfo::operator!=(const TimeOptimalTrajectoryGenerationTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void TimeOptimalTrajectoryGenerationTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimeOptimalTrajectoryGenerationTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TimeOptimalTrajectoryGenerationTaskInfo)

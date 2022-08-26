/**
 * @file ruckig_trajectory_smoothing_task_generator.h
 * @brief Leveraging Ruckig to smooth trajectory
 *
 * @author Levi Armstrong
 * @date July 27, 2022
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/ruckig_trajectory_smoothing_task_generator.h>
#include <tesseract_process_managers/task_profiles/ruckig_trajectory_smoothing_profile.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>

namespace tesseract_planning
{
RuckigTrajectorySmoothingTaskGenerator::RuckigTrajectorySmoothingTaskGenerator(std::string name)
  : TaskGenerator(std::move(name))
{
}

int RuckigTrajectorySmoothingTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<RuckigTrajectorySmoothingTaskInfo>(unique_id, name_);
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
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  auto& ci = input_results->as<CompositeInstruction>();
  CompositeInstruction original_ci{ ci };
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = input.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<RuckigTrajectorySmoothingCompositeProfile>(
      name_, profile, *input.profiles, std::make_shared<RuckigTrajectorySmoothingCompositeProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  RuckigTrajectorySmoothing solver(cur_composite_profile->duration_extension_fraction,
                                   cur_composite_profile->max_duration_extension_factor);

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    CONSOLE_BRIDGE_logWarn("Ruckig trajectory smoothing found no MoveInstructions to process");
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                             cur_composite_profile->max_velocity_scaling_factor;
  Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                 cur_composite_profile->max_acceleration_scaling_factor;
  Eigen::VectorXd jerk_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                         cur_composite_profile->max_jerk_scaling_factor;

  // Loop over all MoveInstructions
  for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
  {
    const auto& mi = flattened[static_cast<std::size_t>(idx)].get().as<MoveInstructionPoly>();
    std::string plan_profile = mi.getProfile();

    // Check for remapping of the plan profile
    plan_profile = getProfileString(name_, profile, input.plan_profile_remapping);
    auto cur_move_profile = getProfile<RuckigTrajectorySmoothingMoveProfile>(
        name_, plan_profile, *input.profiles, std::make_shared<RuckigTrajectorySmoothingMoveProfile>());
    cur_move_profile = applyProfileOverrides(name_, profile, cur_move_profile, mi.getProfileOverrides());

    // If there is a move profile associated with it, override the parameters
    if (cur_move_profile)
    {
      velocity_scaling_factors[idx] = cur_move_profile->max_velocity_scaling_factor;
      acceleration_scaling_factors[idx] = cur_move_profile->max_acceleration_scaling_factor;
      jerk_scaling_factors[idx] = cur_move_profile->max_jerk_scaling_factor;
    }
  }

  // Solve using parameters
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(ci);
  if (!solver.compute(*trajectory,
                      limits.velocity_limits,
                      limits.acceleration_limits,
                      Eigen::VectorXd::Constant(limits.velocity_limits.rows(), 1000),
                      velocity_scaling_factors,
                      acceleration_scaling_factors,
                      jerk_scaling_factors))
  {
    CONSOLE_BRIDGE_logInform("Failed to perform ruckig trajectory smoothing for process input: %s!",
                             input_results->getDescription().c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    // reset back to original
    ci = original_ci;
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Ruckig trajectory smoothing succeeded");
  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

void RuckigTrajectorySmoothingTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

RuckigTrajectorySmoothingTaskInfo::RuckigTrajectorySmoothingTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr RuckigTrajectorySmoothingTaskInfo::clone() const
{
  return std::make_unique<RuckigTrajectorySmoothingTaskInfo>(*this);
}

bool RuckigTrajectorySmoothingTaskInfo::operator==(const RuckigTrajectorySmoothingTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool RuckigTrajectorySmoothingTaskInfo::operator!=(const RuckigTrajectorySmoothingTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void RuckigTrajectorySmoothingTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingTaskInfo)

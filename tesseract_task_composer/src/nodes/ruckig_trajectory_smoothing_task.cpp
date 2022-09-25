/**
 * @file ruckig_trajectory_smoothing_task.h
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
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_task_composer/nodes/ruckig_trajectory_smoothing_task.h>
#include <tesseract_task_composer/profiles/ruckig_trajectory_smoothing_profile.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>
#include <tesseract_time_parameterization/ruckig_trajectory_smoothing.h>

namespace tesseract_planning
{
RuckigTrajectorySmoothingTask::RuckigTrajectorySmoothingTask(std::string input_key,
                                                             std::string output_key,
                                                             bool is_conditional,
                                                             std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr RuckigTrajectorySmoothingTask::runImpl(TaskComposerInput& input,
                                                                  OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  tesseract_common::Timer timer;
  timer.start();

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage.getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input results to ruckig trajectory smoothing must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& ci = input_data_poly.as<CompositeInstruction>();
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = input.problem.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.problem.composite_profile_remapping);
  auto cur_composite_profile = getProfile<RuckigTrajectorySmoothingCompositeProfile>(
      name_, profile, *input.profiles, std::make_shared<RuckigTrajectorySmoothingCompositeProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  RuckigTrajectorySmoothing solver(cur_composite_profile->duration_extension_fraction,
                                   cur_composite_profile->max_duration_extension_factor);

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    info->message = "Ruckig trajectory smoothing found no MoveInstructions to process";
    info->return_value = 1;
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
    return info;
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
    std::string move_profile = mi.getProfile();

    // Check for remapping of the plan profile
    move_profile = getProfileString(name_, profile, input.problem.move_profile_remapping);
    auto cur_move_profile = getProfile<RuckigTrajectorySmoothingMoveProfile>(
        name_, move_profile, *input.profiles, std::make_shared<RuckigTrajectorySmoothingMoveProfile>());
    //    cur_move_profile = applyProfileOverrides(name_, profile, cur_move_profile, mi.profile_overrides);

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
    info->message = "Failed to perform ruckig trajectory smoothing for process input: %s" + ci.getDescription();
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logInform("%s", info->message.c_str());
    return info;
  }

  input.data_storage.setData(output_keys_[0], input_data_poly);
  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  CONSOLE_BRIDGE_logDebug("Ruckig trajectory smoothing succeeded");
  return info;
}

bool RuckigTrajectorySmoothingTask::operator==(const RuckigTrajectorySmoothingTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool RuckigTrajectorySmoothingTask::operator!=(const RuckigTrajectorySmoothingTask& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void RuckigTrajectorySmoothingTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingTask)

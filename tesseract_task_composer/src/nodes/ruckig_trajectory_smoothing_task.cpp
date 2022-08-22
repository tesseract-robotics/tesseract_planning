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
  , input_key_(std::move(input_key))
  , output_key_(std::move(output_key))
{
}

int RuckigTrajectorySmoothingTask::run(TaskComposerInput& input, OptionalTaskComposerExecutor /*executor*/) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<RuckigTrajectorySmoothingTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage->getData(input_key_);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  auto& ci = input_data_poly.as<CompositeInstruction>();
  CompositeInstruction original_ci{ ci };
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = input.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<RuckigTrajectorySmoothingCompositeProfile>(
      name_, profile, *input.profiles, std::make_shared<RuckigTrajectorySmoothingCompositeProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.profile_overrides);

  RuckigTrajectorySmoothing solver(cur_composite_profile->duration_extension_fraction,
                                   cur_composite_profile->max_duration_extension_factor);

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    CONSOLE_BRIDGE_logWarn("Ruckig trajectory smoothing found no MoveInstructions to process");
    info->return_value = 1;
    //    saveOutputs(*info, input);
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
    std::string move_profile = mi.getProfile();

    // Check for remapping of the plan profile
    move_profile = getProfileString(name_, profile, input.move_profile_remapping);
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
    CONSOLE_BRIDGE_logInform("Failed to perform ruckig trajectory smoothing for process input: %s!",
                             ci.getDescription().c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    // reset back to original
    ci = original_ci;
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Ruckig trajectory smoothing succeeded");
  info->return_value = 1;
  //  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

bool RuckigTrajectorySmoothingTask::operator==(const RuckigTrajectorySmoothingTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= (output_key_ == rhs.output_key_);
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
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_NVP(output_key_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

RuckigTrajectorySmoothingTaskInfo::RuckigTrajectorySmoothingTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr RuckigTrajectorySmoothingTaskInfo::clone() const
{
  return std::make_unique<RuckigTrajectorySmoothingTaskInfo>(*this);
}

bool RuckigTrajectorySmoothingTaskInfo::operator==(const RuckigTrajectorySmoothingTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool RuckigTrajectorySmoothingTaskInfo::operator!=(const RuckigTrajectorySmoothingTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void RuckigTrajectorySmoothingTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingTaskInfo)

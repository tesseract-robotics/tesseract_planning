/**
 * @file time_optimal_trajectory_generation_task.cpp
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
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_task_composer/planning/nodes/time_optimal_parameterization_task.h>
#include <tesseract_task_composer/planning/profiles/time_optimal_parameterization_profile.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_time_parameterization/core/utils.h>

namespace tesseract_planning
{
TimeOptimalParameterizationTask::TimeOptimalParameterizationTask()
  : TaskComposerTask("TimeOptimalParameterizationTask", true)
{
}
TimeOptimalParameterizationTask::TimeOptimalParameterizationTask(std::string name,
                                                                 std::string input_key,
                                                                 std::string output_key,
                                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

TimeOptimalParameterizationTask::TimeOptimalParameterizationTask(std::string name,
                                                                 const YAML::Node& config,
                                                                 const TaskComposerPluginFactory&)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("TimeOptimalParameterizationTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("TimeOptimalParameterizationTask, config 'inputs' entry currently only supports one "
                             "input key");

  if (output_keys_.empty())
    throw std::runtime_error("TimeOptimalParameterizationTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("TimeOptimalParameterizationTask, config 'outputs' entry currently only supports one "
                             "output key");
}

TaskComposerNodeInfo::UPtr TimeOptimalParameterizationTask::runImpl(TaskComposerContext& context,
                                                                    OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<TimeOptimalParameterizationTaskInfo>(*this);
  info->return_value = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input results to TOTG must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& ci = input_data_poly.as<CompositeInstruction>();
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = problem.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, problem.composite_profile_remapping);
  auto cur_composite_profile = getProfile<TimeOptimalParameterizationProfile>(
      name_, profile, *problem.profiles, std::make_shared<TimeOptimalParameterizationProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_[0] != input_keys_[0])
      context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

    info->color = "green";
    info->message = "TOTG found no MoveInstructions to process";
    info->return_value = 1;
    CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
    return info;
  }

  // Solve using parameters
  TimeOptimalTrajectoryGeneration solver(cur_composite_profile->path_tolerance,
                                         cur_composite_profile->min_angle_change);

  // Store scaling factors
  info->max_velocity_scaling_factor = cur_composite_profile->max_velocity_scaling_factor;
  info->max_acceleration_scaling_factor = cur_composite_profile->max_acceleration_scaling_factor;

  // Copy the Composite before passing in because it will get flattened and resampled
  CompositeInstruction copy_ci(ci);
  InstructionsTrajectory traj_wrapper(copy_ci);
  if (!solver.computeTimeStamps(traj_wrapper,
                                limits.velocity_limits,
                                limits.acceleration_limits,
                                cur_composite_profile->max_velocity_scaling_factor,
                                cur_composite_profile->max_acceleration_scaling_factor))
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_[0] != input_keys_[0])
      context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

    info->message = "Failed to perform TOTG for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info->message.c_str());
    return info;
  }

  context.data_storage->setData(output_keys_[0], copy_ci);

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("TOTG succeeded");
  return info;
}

bool TimeOptimalParameterizationTask::operator==(const TimeOptimalParameterizationTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool TimeOptimalParameterizationTask::operator!=(const TimeOptimalParameterizationTask& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void TimeOptimalParameterizationTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

TimeOptimalParameterizationTaskInfo::TimeOptimalParameterizationTaskInfo(const TimeOptimalParameterizationTask& task)
  : TaskComposerNodeInfo(task)
{
}

TaskComposerNodeInfo::UPtr TimeOptimalParameterizationTaskInfo::clone() const
{
  return std::make_unique<TimeOptimalParameterizationTaskInfo>(*this);
}

bool TimeOptimalParameterizationTaskInfo::operator==(const TimeOptimalParameterizationTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  equal &= tesseract_common::almostEqualRelativeAndAbs(max_velocity_scaling_factor, rhs.max_velocity_scaling_factor);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(max_acceleration_scaling_factor, rhs.max_acceleration_scaling_factor);
  return equal;
}
bool TimeOptimalParameterizationTaskInfo::operator!=(const TimeOptimalParameterizationTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void TimeOptimalParameterizationTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimeOptimalParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TimeOptimalParameterizationTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimeOptimalParameterizationTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TimeOptimalParameterizationTaskInfo)

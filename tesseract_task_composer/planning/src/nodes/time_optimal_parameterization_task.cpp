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

#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/time_optimal_parameterization_task.h>
#include <tesseract_task_composer/planning/profiles/time_optimal_parameterization_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_time_parameterization/core/utils.h>

namespace tesseract_planning
{
// Requried
const std::string TimeOptimalParameterizationTask::INOUT_PROGRAM_PORT = "program";
const std::string TimeOptimalParameterizationTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string TimeOptimalParameterizationTask::INPUT_PROFILES_PORT = "profiles";

// Optional
const std::string TimeOptimalParameterizationTask::INPUT_MANIP_INFO_PORT = "manip_info";

TimeOptimalParameterizationTask::TimeOptimalParameterizationTask()
  : TaskComposerTask("TimeOptimalParameterizationTask", TimeOptimalParameterizationTask::ports(), true)
{
}
TimeOptimalParameterizationTask::TimeOptimalParameterizationTask(std::string name,
                                                                 std::string input_program_key,
                                                                 std::string input_environment_key,
                                                                 std::string input_profiles_key,
                                                                 std::string output_program_key,
                                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), TimeOptimalParameterizationTask::ports(), is_conditional)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

TimeOptimalParameterizationTask::TimeOptimalParameterizationTask(std::string name,
                                                                 const YAML::Node& config,
                                                                 const TaskComposerPluginFactory&)
  : TaskComposerTask(std::move(name), TimeOptimalParameterizationTask::ports(), config)
{
}

TaskComposerNodePorts TimeOptimalParameterizationTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.input_optional[INPUT_MANIP_INFO_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

std::unique_ptr<TaskComposerNodeInfo>
TimeOptimalParameterizationTask::runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
  {
    info->status_code = 0;
    info->status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    info->return_value = 0;
    return info;
  }

  auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

  auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_message = "Input results to TOTG must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }
  tesseract_common::AnyPoly original_input_data_poly{ input_data_poly };

  tesseract_common::ManipulatorInfo input_manip_info;
  auto manip_info_poly = getData(*context.data_storage, INPUT_MANIP_INFO_PORT, false);
  if (!manip_info_poly.isNull())
    input_manip_info = manip_info_poly.as<tesseract_common::ManipulatorInfo>();

  auto& ci = input_data_poly.as<CompositeInstruction>();
  tesseract_common::ManipulatorInfo manip_info = ci.getManipulatorInfo().getCombined(input_manip_info);
  auto joint_group = env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<ProfileDictionary>>();
  auto cur_composite_profile = getProfile<TimeOptimalParameterizationProfile>(
      ns_, ci.getProfile(ns_), *profiles, std::make_shared<TimeOptimalParameterizationProfile>());

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

    info->color = "green";
    info->status_code = 1;
    info->status_message = "TOTG found no MoveInstructions to process";
    info->return_value = 1;
    CONSOLE_BRIDGE_logWarn("%s", info->status_message.c_str());
    return info;
  }

  // Solve using parameters
  TimeOptimalTrajectoryGeneration solver(cur_composite_profile->path_tolerance,
                                         cur_composite_profile->min_angle_change);

  // Store scaling factors
  info->data_storage.setData("max_velocity_scaling_factor", cur_composite_profile->max_velocity_scaling_factor);
  info->data_storage.setData("max_acceleration_scaling_factor", cur_composite_profile->max_acceleration_scaling_factor);

  // Copy the Composite before passing in because it will get flattened and resampled
  CompositeInstruction copy_ci(ci);
  InstructionsTrajectory traj_wrapper(copy_ci);
  if (!solver.compute(traj_wrapper,
                      limits.velocity_limits,
                      limits.acceleration_limits,
                      limits.jerk_limits,
                      Eigen::VectorXd::Constant(1, cur_composite_profile->max_velocity_scaling_factor),
                      Eigen::VectorXd::Constant(1, cur_composite_profile->max_acceleration_scaling_factor),
                      Eigen::VectorXd::Constant(1, cur_composite_profile->max_jerk_scaling_factor)))
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

    info->status_message = "Failed to perform TOTG for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info->status_message.c_str());
    return info;
  }

  setData(*context.data_storage, INOUT_PROGRAM_PORT, copy_ci);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
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

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimeOptimalParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TimeOptimalParameterizationTask)

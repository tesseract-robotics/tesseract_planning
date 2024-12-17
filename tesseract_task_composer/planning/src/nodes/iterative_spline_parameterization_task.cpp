/**
 * @file iterative_spline_parameterization_task.cpp
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
#include <yaml-cpp/yaml.h>
#include <boost/serialization/string.hpp>

#include <tesseract_common/serialization.h>

#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

namespace tesseract_planning
{
// Requried
const std::string IterativeSplineParameterizationTask::INOUT_PROGRAM_PORT = "program";
const std::string IterativeSplineParameterizationTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string IterativeSplineParameterizationTask::INPUT_PROFILES_PORT = "profiles";

// Optional
const std::string IterativeSplineParameterizationTask::INPUT_MANIP_INFO_PORT = "manip_info";

IterativeSplineParameterizationTask::IterativeSplineParameterizationTask()
  : TaskComposerTask("IterativeSplineParameterizationTask", IterativeSplineParameterizationTask::ports(), true)
{
}
IterativeSplineParameterizationTask::IterativeSplineParameterizationTask(std::string name,
                                                                         std::string input_program_key,
                                                                         std::string input_environment_key,
                                                                         std::string input_profiles_key,
                                                                         std::string output_program_key,
                                                                         bool is_conditional,
                                                                         bool add_points)
  : TaskComposerTask(std::move(name), IterativeSplineParameterizationTask::ports(), is_conditional)
  , add_points_(add_points)
  , solver_(add_points)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

IterativeSplineParameterizationTask::IterativeSplineParameterizationTask(
    std::string name,
    const YAML::Node& config,
    const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), IterativeSplineParameterizationTask::ports(), config)
{
  try
  {
    if (YAML::Node n = config["add_points"])
      add_points_ = n.as<bool>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("IterativeSplineParameterizationTask: Failed to parse yaml config data! Details: " +
                             std::string(e.what()));
  }
}

TaskComposerNodePorts IterativeSplineParameterizationTask::ports()
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
IterativeSplineParameterizationTask::runImpl(TaskComposerContext& context,
                                             OptionalTaskComposerExecutor /*executor*/) const
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
    info->status_message = "Input results to iterative spline parameterization must be a composite instruction";
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
  std::string profile = ci.getProfile();
  auto cur_composite_profile = getProfile<IterativeSplineParameterizationProfile>(
      ns_, ci.getProfile(ns_), *profiles, std::make_shared<IterativeSplineParameterizationProfile>());

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
    info->status_message = "Iterative spline time parameterization found no MoveInstructions to process";
    info->return_value = 1;
    CONSOLE_BRIDGE_logWarn("%s", info->status_message.c_str());
    return info;
  }

  Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                             cur_composite_profile->max_velocity_scaling_factor;
  Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                 cur_composite_profile->max_acceleration_scaling_factor;
  Eigen::VectorXd jerk_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size()));

  // Loop over all MoveInstructions
  for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
  {
    const auto& mi = flattened[static_cast<std::size_t>(idx)].get().as<MoveInstructionPoly>();

    // Check for remapping of the plan profil
    auto cur_move_profile = getProfile<IterativeSplineParameterizationProfile>(
        ns_, mi.getProfile(ns_), *profiles, std::make_shared<IterativeSplineParameterizationProfile>());

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
                       limits.jerk_limits,
                       velocity_scaling_factors,
                       acceleration_scaling_factors,
                       jerk_scaling_factors))
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

    info->status_message =
        "Failed to perform iterative spline time parameterization for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info->status_message.c_str());
    return info;
  }

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  setData(*context.data_storage, INOUT_PROGRAM_PORT, input_data_poly);
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  return info;
}

bool IterativeSplineParameterizationTask::operator==(const IterativeSplineParameterizationTask& rhs) const
{
  bool equal = true;
  equal &= (add_points_ == rhs.add_points_);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool IterativeSplineParameterizationTask::operator!=(const IterativeSplineParameterizationTask& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void IterativeSplineParameterizationTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(add_points_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::IterativeSplineParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::IterativeSplineParameterizationTask)

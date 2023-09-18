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
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

namespace tesseract_planning
{
IterativeSplineParameterizationTask::IterativeSplineParameterizationTask()
  : TaskComposerTask("IterativeSplineParameterizationTask", true)
{
}
IterativeSplineParameterizationTask::IterativeSplineParameterizationTask(std::string name,
                                                                         std::string input_key,
                                                                         std::string output_key,
                                                                         bool is_conditional,
                                                                         bool add_points)
  : TaskComposerTask(std::move(name), is_conditional), add_points_(add_points), solver_(add_points)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

IterativeSplineParameterizationTask::IterativeSplineParameterizationTask(
    std::string name,
    const YAML::Node& config,
    const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("IterativeSplineParameterizationTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("IterativeSplineParameterizationTask, config 'inputs' entry currently only supports "
                             "one input key");

  if (output_keys_.empty())
    throw std::runtime_error("IterativeSplineParameterizationTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("IterativeSplineParameterizationTask, config 'outputs' entry currently only supports "
                             "one output key");

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

TaskComposerNodeInfo::UPtr IterativeSplineParameterizationTask::runImpl(TaskComposerContext& context,
                                                                        OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input results to iterative spline parameterization must be a composite instruction";
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
  auto cur_composite_profile = getProfile<IterativeSplineParameterizationProfile>(
      name_, profile, *problem.profiles, std::make_shared<IterativeSplineParameterizationProfile>());
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
    info->message = "Iterative spline time parameterization found no MoveInstructions to process";
    info->return_value = 1;
    CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
    return info;
  }

  Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                             cur_composite_profile->max_velocity_scaling_factor;
  Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                 cur_composite_profile->max_acceleration_scaling_factor;

  // Loop over all MoveInstructions
  for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
  {
    const auto& mi = flattened[static_cast<std::size_t>(idx)].get().as<MoveInstructionPoly>();
    std::string move_profile = mi.getProfile();

    // Check for remapping of the plan profile
    move_profile = getProfileString(name_, profile, problem.move_profile_remapping);
    auto cur_move_profile = getProfile<IterativeSplineParameterizationProfile>(
        name_, move_profile, *problem.profiles, std::make_shared<IterativeSplineParameterizationProfile>());
    //    cur_move_profile = applyProfileOverrides(name_, profile, cur_move_profile, mi.profile_overrides);

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
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_[0] != input_keys_[0])
      context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

    info->message =
        "Failed to perform iterative spline time parameterization for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info->message.c_str());
    return info;
  }

  info->color = "green";
  info->message = "Successful";
  context.data_storage->setData(output_keys_[0], input_data_poly);
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

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::IterativeSplineParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::IterativeSplineParameterizationTask)

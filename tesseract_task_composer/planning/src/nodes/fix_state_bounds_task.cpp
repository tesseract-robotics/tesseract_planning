/**
 * @file fix_state_bounds_task.cpp
 * @brief Task that changes the plan instructions to make push them back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/fix_state_bounds_task.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
// Requried
const std::string FixStateBoundsTask::INOUT_PROGRAM_PORT = "program";
const std::string FixStateBoundsTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string FixStateBoundsTask::INPUT_PROFILES_PORT = "profiles";

// Optional
const std::string FixStateBoundsTask::INPUT_MANIP_INFO_PORT = "manip_info";

FixStateBoundsTask::FixStateBoundsTask() : TaskComposerTask("FixStateBoundsTask", FixStateBoundsTask::ports(), true) {}
FixStateBoundsTask::FixStateBoundsTask(std::string name,
                                       std::string input_program_key,
                                       std::string input_environment_key,
                                       std::string input_profiles_key,
                                       std::string output_program_key,
                                       bool is_conditional)
  : TaskComposerTask(std::move(name), FixStateBoundsTask::ports(), is_conditional)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

FixStateBoundsTask::FixStateBoundsTask(std::string name,
                                       const YAML::Node& config,
                                       const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), FixStateBoundsTask::ports(), config)
{
}

TaskComposerNodePorts FixStateBoundsTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.input_optional[INPUT_MANIP_INFO_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

std::unique_ptr<TaskComposerNodeInfo> FixStateBoundsTask::runImpl(TaskComposerContext& context,
                                                                  OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  tesseract_common::AnyPoly env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
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
    info->status_message = "Input instruction to FixStateBounds must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  tesseract_common::AnyPoly original_input_data_poly{ input_data_poly };

  tesseract_common::ManipulatorInfo input_manip_info;
  auto manip_info_poly = getData(*context.data_storage, INPUT_MANIP_INFO_PORT, false);
  if (!manip_info_poly.isNull())
    input_manip_info = manip_info_poly.as<tesseract_common::ManipulatorInfo>();

  auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<ProfileDictionary>>();
  auto& ci = input_data_poly.as<CompositeInstruction>();
  ci.setManipulatorInfo(ci.getManipulatorInfo().getCombined(input_manip_info));
  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();
  auto joint_group = env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  auto cur_composite_profile =
      getProfile<FixStateBoundsProfile>(ns_, ci.getProfile(ns_), *profiles, std::make_shared<FixStateBoundsProfile>());

  limits.joint_limits.col(0) = limits.joint_limits.col(0).array() + cur_composite_profile->lower_bounds_reduction;
  limits.joint_limits.col(1) = limits.joint_limits.col(1).array() - cur_composite_profile->upper_bounds_reduction;
  switch (cur_composite_profile->mode)
  {
    case FixStateBoundsProfile::Settings::START_ONLY:
    {
      MoveInstructionPoly* first_mi = ci.getFirstMoveInstruction();
      if (first_mi != nullptr)
      {
        auto& wp = first_mi->getWaypoint();
        if (wp.isStateWaypoint() || wp.isJointWaypoint())
        {
          if (!isWithinJointLimits(wp, limits.joint_limits))
          {
            CONSOLE_BRIDGE_logInform("FixStateBoundsTask is modifying the input instructions");
            if (!clampToJointLimits(wp, limits.joint_limits, cur_composite_profile->max_deviation_global))
            {
              // If the output key is not the same as the input key the output data should be assigned the input data
              // for error branching
              if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
                setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

              info->status_message = "Failed to clamp to joint limits";
              return info;
            }
          }
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::END_ONLY:
    {
      MoveInstructionPoly* last_mi = ci.getLastMoveInstruction();
      if (last_mi != nullptr)
      {
        auto& wp = last_mi->getWaypoint();
        if (wp.isStateWaypoint() || wp.isJointWaypoint())
        {
          if (!isWithinJointLimits(wp, limits.joint_limits))
          {
            CONSOLE_BRIDGE_logInform("FixStateBoundsTask is modifying the input instructions");
            if (!clampToJointLimits(wp, limits.joint_limits, cur_composite_profile->max_deviation_global))
            {
              // If the output key is not the same as the input key the output data should be assigned the input data
              // for error branching
              if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
                setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

              info->status_message = "Failed to clamp to joint limits";
              return info;
            }
          }
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::ALL:
    {
      auto flattened = ci.flatten(moveFilter);
      if (flattened.empty())
      {
        // If the output key is not the same as the input key the output data should be assigned the input data for
        // error branching
        if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
          setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

        info->color = "green";
        info->status_code = 1;
        info->status_message = "FixStateBoundsTask found no MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->status_message.c_str());
        return info;
      }

      bool inside_limits = true;
      for (const auto& instruction : flattened)
      {
        const auto& wp = instruction.get().as<MoveInstructionPoly>().getWaypoint();
        if (wp.isStateWaypoint() || wp.isJointWaypoint())
          inside_limits &= isWithinJointLimits(wp, limits.joint_limits);
      }
      if (inside_limits)
        break;

      CONSOLE_BRIDGE_logInform("FixStateBoundsTask is modifying the input instructions");
      for (auto& instruction : flattened)
      {
        auto& wp = instruction.get().as<MoveInstructionPoly>().getWaypoint();
        if (wp.isStateWaypoint() || wp.isJointWaypoint())
        {
          if (!clampToJointLimits(wp, limits.joint_limits, cur_composite_profile->max_deviation_global))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
              setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

            info->status_message = "Failed to clamp to joint limits";
            return info;
          }
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::DISABLED:
    {
      // If the output key is not the same as the input key the output data should be assigned the input data for
      // error branching
      if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
        setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

      info->color = "yellow";
      info->status_code = 1;
      info->status_message = "Successful, DISABLED";
      info->return_value = 1;
      return info;
    }
  }

  setData(*context.data_storage, INOUT_PROGRAM_PORT, input_data_poly);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("FixStateBoundsTask succeeded");
  return info;
}

bool FixStateBoundsTask::operator==(const FixStateBoundsTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool FixStateBoundsTask::operator!=(const FixStateBoundsTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void FixStateBoundsTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateBoundsTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateBoundsTask)

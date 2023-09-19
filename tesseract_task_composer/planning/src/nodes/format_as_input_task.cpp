/**
 * @file format_as_input_task.h
 *
 * @brief This is used in the case where you run trajopt with collision as a cost and then you post check it for
 * collision and it fails. Then you run trajopt with collision as a constraint but the output from trajopt with
 * collision as a cost must be formated as input for trajopt with collision as a constraint planner.
 *
 * @author Levi Armstrong
 * @date April 6. 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>

namespace tesseract_planning
{
FormatAsInputTask::FormatAsInputTask() : TaskComposerTask("FormatAsInputTask", true) {}
FormatAsInputTask::FormatAsInputTask(std::string name,
                                     const std::array<std::string, 2>& input_keys,
                                     std::string output_key,
                                     bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.reserve(2);
  input_keys_.insert(input_keys_.end(), input_keys.begin(), input_keys.end());
  output_keys_.push_back(std::move(output_key));
}

FormatAsInputTask::FormatAsInputTask(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("FormatAsInputTask, config missing 'inputs' entry");

  if (input_keys_.size() != 2)
    throw std::runtime_error("FormatAsInputTask, config 'inputs' entry requires two input key");

  if (output_keys_.empty())
    throw std::runtime_error("FormatAsInputTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("FormatAsInputTask, config 'outputs' entry requires one output key");
}

TaskComposerNodeInfo::UPtr FormatAsInputTask::runImpl(TaskComposerContext& context,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_formatted_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_formatted_data_poly.isNull() ||
      input_formatted_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input[0] instruction to FormatAsInputTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto input_unformatted_data_poly = context.data_storage->getData(input_keys_[1]);
  if (input_unformatted_data_poly.isNull() ||
      input_unformatted_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input[1] instruction to FormatAsInputTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& ci_formatted_data = input_formatted_data_poly.as<CompositeInstruction>();
  const auto& ci_unformatted_data = input_unformatted_data_poly.as<CompositeInstruction>();

  std::vector<std::reference_wrapper<InstructionPoly>> mi_formatted_data = ci_formatted_data.flatten();
  std::vector<std::reference_wrapper<const InstructionPoly>> mi_unformatted_data =
      ci_unformatted_data.flatten(&moveFilter);

  if (mi_formatted_data.size() != mi_unformatted_data.size())
  {
    info->message = "FormatAsInputTask, input programs are not same size";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  for (std::size_t i = 0; i < mi_formatted_data.size(); ++i)
  {
    auto& mi = mi_formatted_data[i].get().as<MoveInstructionPoly>();
    const auto& umi = mi_unformatted_data[i].get().as<MoveInstructionPoly>();

    if (mi.getWaypoint().isStateWaypoint())
      continue;

    if (mi.getWaypoint().isCartesianWaypoint())
    {
      const auto& swp = umi.getWaypoint().as<StateWaypointPoly>();
      auto& cwp = mi.getWaypoint().as<CartesianWaypointPoly>();
      cwp.setSeed(tesseract_common::JointState(swp.getNames(), swp.getPosition()));
    }
    else if (mi.getWaypoint().isJointWaypoint())
    {
      auto& jwp = mi.getWaypoint().as<JointWaypointPoly>();
      if (!jwp.isConstrained() || (jwp.isConstrained() && jwp.isToleranced()))
      {
        const auto& swp = umi.getWaypoint().as<StateWaypointPoly>();
        jwp.setNames(swp.getNames());
        jwp.setPosition(swp.getPosition());
      }
    }
    else
    {
      throw std::runtime_error("FormatAsInputTask, unsupported waypoint type!");
    }
  }

  context.data_storage->setData(output_keys_[0], input_formatted_data_poly);

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  return info;
}

bool FormatAsInputTask::operator==(const FormatAsInputTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool FormatAsInputTask::operator!=(const FormatAsInputTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void FormatAsInputTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FormatAsInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FormatAsInputTask)

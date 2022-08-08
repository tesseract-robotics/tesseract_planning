/**
 * @file motion_planner_node.h
 * @brief Task Composer motion planner node
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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

#include <tesseract_task_composer/nodes/motion_planner_task.h>

namespace tesseract_planning
{
MotionPlannerTask::MotionPlannerTask(MotionPlanner::Ptr planner,
                                     std::string input_key,
                                     std::string output_key,
                                     bool format_result_as_input)
  : TaskComposerNode(planner->getName())
  , planner_(std::move(planner))
  , input_key_(std::move(input_key))
  , output_key_(std::move(output_key))
  , format_result_as_input_(format_result_as_input)
{
}

int MotionPlannerTask::run(TaskComposerInput& input) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<MotionPlannerTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  auto input_data_poly = input.data_storage->getData(input_key_);
  auto result_data_poly = input.data_storage->getData(output_key_);

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input instructions to MotionPlannerTask: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  if (result_data_poly.isNull() || result_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input seed to MotionPlannerTask: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Make a non-const copy of the input instructions to update the start/end
  CompositeInstruction& instructions = input_data_poly.as<CompositeInstruction>();
  assert(!(input.manip_info.empty() && input.manip_info.empty()));
  instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(input.manip_info));

  // It should always have a start instruction which required by the motion planners
  assert(instructions.hasStartInstruction());

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = result_data_poly.as<CompositeInstruction>();
  request.env_state = input.env->getState();
  request.env = input.env;
  request.instructions = instructions;
  request.profiles = input.profiles;
  request.plan_profile_remapping = input.move_profile_remapping;
  request.composite_profile_remapping = input.composite_profile_remapping;
  request.format_result_as_input = format_result_as_input_;

  // --------------------
  // Fill out response
  // --------------------
  request.verbose = false;
  if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
    request.verbose = true;
  PlannerResponse response = planner_->solve(request);

  // --------------------
  // Verify Success
  // --------------------
  if (response)
  {
    input.data_storage->setData("output_key_", response.results);

    CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
    info->return_value = 1;
    info->message = response.message;
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                           planner_->getName().c_str(),
                           response.message.c_str(),
                           instructions.getDescription().c_str());
  info->message = response.message;
  //  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 0;
}

bool MotionPlannerTask::operator==(const MotionPlannerTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= (output_key_ == rhs.output_key_);
  equal &= (format_result_as_input_ == rhs.format_result_as_input_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool MotionPlannerTask::operator!=(const MotionPlannerTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void MotionPlannerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_NVP(output_key_);
  ar& BOOST_SERIALIZATION_NVP(format_result_as_input_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

MotionPlannerTaskInfo::MotionPlannerTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr MotionPlannerTaskInfo::clone() const
{
  return std::make_unique<MotionPlannerTaskInfo>(*this);
}

bool MotionPlannerTaskInfo::operator==(const MotionPlannerTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool MotionPlannerTaskInfo::operator!=(const MotionPlannerTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void MotionPlannerTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MotionPlannerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MotionPlannerTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MotionPlannerTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MotionPlannerTaskInfo)

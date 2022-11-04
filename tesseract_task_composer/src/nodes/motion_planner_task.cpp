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
                                     bool format_result_as_input,
                                     bool is_conditional)
  : TaskComposerTask(is_conditional, planner->getName())
  , planner_(std::move(planner))
  , format_result_as_input_(format_result_as_input)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr MotionPlannerTask::runImpl(TaskComposerInput& input,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->env = input.problem.env;

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
    info->message = "Input instructions to MotionPlannerTask: " + name_ + " must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Make a non-const copy of the input instructions to update the start/end
  auto& instructions = input_data_poly.as<CompositeInstruction>();
  assert(!(input.problem.manip_info.empty() && instructions.getManipulatorInfo().empty()));
  instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(input.problem.manip_info));

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.env_state = input.problem.env->getState();
  request.env = input.problem.env;
  request.instructions = instructions;
  request.profiles = input.profiles;
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
    input.data_storage.setData(output_keys_[0], response.results);

    info->return_value = 1;
    info->message = response.message;
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
    return info;
  }

  CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                           planner_->getName().c_str(),
                           response.message.c_str(),
                           instructions.getDescription().c_str());
  info->message = response.message;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

bool MotionPlannerTask::operator==(const MotionPlannerTask& rhs) const
{
  bool equal = true;
  equal &= (format_result_as_input_ == rhs.format_result_as_input_);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool MotionPlannerTask::operator!=(const MotionPlannerTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void MotionPlannerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(format_result_as_input_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MotionPlannerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MotionPlannerTask)

/**
 * @file min_length_task.cpp
 * @brief Task for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>

#include <tesseract/common/profile_dictionary.h>
#include <tesseract/environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/min_length_task.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>

namespace tesseract::task_composer
{
// Requried
const std::string MinLengthTask::INOUT_PROGRAM_PORT = "program";
const std::string MinLengthTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string MinLengthTask::INPUT_PROFILES_PORT = "profiles";

MinLengthTask::MinLengthTask() : TaskComposerTask("MinLengthTask", MinLengthTask::ports(), false) {}
MinLengthTask::MinLengthTask(std::string name,
                             std::string input_program_key,
                             std::string input_environment_key,
                             std::string input_profiles_key,
                             std::string output_program_key,
                             bool is_conditional)
  : TaskComposerTask(std::move(name), MinLengthTask::ports(), is_conditional)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

MinLengthTask::MinLengthTask(std::string name,
                             const YAML::Node& config,
                             const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), MinLengthTask::ports(), config)
{
}

TaskComposerNodePorts MinLengthTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

TaskComposerNodeInfo MinLengthTask::runImpl(TaskComposerContext& context,
                                            OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(context, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract::environment::Environment>)))
  {
    info.status_code = 0;
    info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    info.return_value = 0;
    return info;
  }

  auto env = env_poly.as<std::shared_ptr<const tesseract::environment::Environment>>();

  auto input_data_poly = getData(context, INOUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(tesseract::command_language::CompositeInstruction)))
  {
    info.status_message = "Input seed to MinLengthTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  // Get Composite Profile
  auto profiles = getData(context, INPUT_PROFILES_PORT).as<std::shared_ptr<tesseract::common::ProfileDictionary>>();
  const auto& ci = input_data_poly.as<tesseract::command_language::CompositeInstruction>();
  long cnt = ci.getMoveInstructionCount();
  auto cur_composite_profile =
      profiles->getProfile<MinLengthProfile>(ns_, ci.getProfile(ns_), std::make_shared<MinLengthProfile>());

  if (cnt < cur_composite_profile->min_length)
  {
    auto subdivisions = static_cast<int>(
        std::ceil(static_cast<double>(cur_composite_profile->min_length) / static_cast<double>(cnt - 1)));

    // Fill out request and response
    tesseract::motion_planners::PlannerRequest request;
    request.instructions = ci;
    request.env = env;
    request.format_result_as_input = true;

    // Set up planner
    tesseract::motion_planners::SimpleMotionPlanner planner(ns_);

    auto profile =
        std::make_shared<tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile>(subdivisions, subdivisions);

    // Create profile dictionary
    auto simple_profiles = std::make_shared<tesseract::common::ProfileDictionary>();
    simple_profiles->addProfile(planner.getName(), ci.getProfile(), profile);
    auto flat = ci.flatten(&tesseract::command_language::moveFilter);
    for (const auto& i : flat)
      simple_profiles->addProfile(
          planner.getName(), i.get().as<tesseract::command_language::MoveInstructionPoly>().getProfile(), profile);

    // Assign profile dictionary
    request.profiles = simple_profiles;

    // Solve
    tesseract::motion_planners::PlannerResponse response = planner.solve(request);

    if (!response.successful)
    {
      info.status_message = "MinLengthTask, failed to subdivid!";
      CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
      return info;
    }

    setData(context, INOUT_PROGRAM_PORT, response.results);
  }
  else
  {
    setData(context, INOUT_PROGRAM_PORT, ci);
  }

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  CONSOLE_BRIDGE_logDebug("Seed Min Length Task Succeeded!");
  return info;
}

}  // namespace tesseract::task_composer

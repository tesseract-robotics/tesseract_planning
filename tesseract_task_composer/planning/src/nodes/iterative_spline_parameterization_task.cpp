/**
 * @file iterative_spline_parameterization_task.cpp
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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

#include <tesseract_common/profile_dictionary.h>
#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

namespace tesseract::task_composer
{
// Requried
const std::string IterativeSplineParameterizationTask::INOUT_PROGRAM_PORT = "program";
const std::string IterativeSplineParameterizationTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string IterativeSplineParameterizationTask::INPUT_PROFILES_PORT = "profiles";

IterativeSplineParameterizationTask::IterativeSplineParameterizationTask()
  : TaskComposerTask("IterativeSplineParameterizationTask", IterativeSplineParameterizationTask::ports(), true)
  , solver_(ns_)
{
}
IterativeSplineParameterizationTask::IterativeSplineParameterizationTask(std::string name,
                                                                         std::string input_program_key,
                                                                         std::string input_environment_key,
                                                                         std::string input_profiles_key,
                                                                         std::string output_program_key,
                                                                         bool is_conditional)
  : TaskComposerTask(std::move(name), IterativeSplineParameterizationTask::ports(), is_conditional), solver_(ns_)
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
  : TaskComposerTask(std::move(name), IterativeSplineParameterizationTask::ports(), config), solver_(ns_)
{
}

TaskComposerNodePorts IterativeSplineParameterizationTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

TaskComposerNodeInfo IterativeSplineParameterizationTask::runImpl(TaskComposerContext& context,
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
    info.status_message = "Input results to iterative spline parameterization must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }
  tesseract::common::AnyPoly original_input_data_poly{ input_data_poly };

  // Get Composite Profile
  auto profiles = getData(context, INPUT_PROFILES_PORT).as<std::shared_ptr<tesseract::common::ProfileDictionary>>();

  auto& ci = input_data_poly.as<tesseract::command_language::CompositeInstruction>();
  if (ci.getMoveInstructionCount() == 0)
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(context, INOUT_PROGRAM_PORT, original_input_data_poly);

    info.color = "green";
    info.status_code = 1;
    info.status_message = "Iterative spline time parameterization found no MoveInstructions to process";
    info.return_value = 1;
    CONSOLE_BRIDGE_logWarn("%s", info.status_message.c_str());
    return info;
  }

  // Solve using parameters
  if (!solver_.compute(ci, *env, *profiles))
  {
    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(context, INOUT_PROGRAM_PORT, original_input_data_poly);

    info.status_message =
        "Failed to perform iterative spline time parameterization for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info.status_message.c_str());
    return info;
  }

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  setData(context, INOUT_PROGRAM_PORT, input_data_poly);
  info.return_value = 1;
  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  return info;
}

}  // namespace tesseract::task_composer

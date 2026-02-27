/**
 * @file discrete_contact_check_task.cpp
 * @brief Discrete collision check trajectory
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract::task_composer
{
// Requried
const std::string DiscreteContactCheckTask::INPUT_PROGRAM_PORT = "program";
const std::string DiscreteContactCheckTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string DiscreteContactCheckTask::INPUT_PROFILES_PORT = "profiles";

// Optional
const std::string DiscreteContactCheckTask::OUTPUT_CONTACT_RESULTS_PORT = "contact_results";

DiscreteContactCheckTask::DiscreteContactCheckTask()
  : TaskComposerTask("DiscreteContactCheckTask", DiscreteContactCheckTask::ports(), true)
{
}

DiscreteContactCheckTask::DiscreteContactCheckTask(std::string name,
                                                   std::string input_program_key,
                                                   std::string input_environment_key,
                                                   std::string input_profiles_key,
                                                   bool is_conditional)
  : TaskComposerTask(std::move(name), DiscreteContactCheckTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  validatePorts();
}

DiscreteContactCheckTask::DiscreteContactCheckTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), DiscreteContactCheckTask::ports(), config)
{
}

TaskComposerNodePorts DiscreteContactCheckTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_optional[OUTPUT_CONTACT_RESULTS_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

TaskComposerNodeInfo DiscreteContactCheckTask::runImpl(TaskComposerContext& context,
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

  auto input_data_poly = getData(context, INPUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(tesseract::command_language::CompositeInstruction)))
  {
    info.status_message = "Input to DiscreteContactCheckTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  // Get Composite Profile
  auto profiles = getData(context, INPUT_PROFILES_PORT).as<std::shared_ptr<tesseract::common::ProfileDictionary>>();
  const auto& ci = input_data_poly.as<tesseract::command_language::CompositeInstruction>();
  auto cur_composite_profile =
      profiles->getProfile<ContactCheckProfile>(ns_, ci.getProfile(ns_), std::make_shared<ContactCheckProfile>());

  // Get state solver
  tesseract::common::ManipulatorInfo manip_info = ci.getManipulatorInfo();
  tesseract::kinematics::JointGroup::ConstPtr manip = env->getJointGroup(manip_info.manipulator);
  tesseract::scene_graph::StateSolver::UPtr state_solver = env->getStateSolver();
  tesseract::collision::DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->applyContactManagerConfig(cur_composite_profile->contact_manager_config);

  std::vector<tesseract::collision::ContactResultMap> contacts;
  tesseract::collision::ContactTrajectoryResults traj_results = tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, ci, cur_composite_profile->collision_check_config);
  info.status_message = traj_results.condensedSummary().str();
  if (traj_results)
  {
    info.status_code = 0;
    CONSOLE_BRIDGE_logInform("%s", info.status_message.c_str());

    // Save space
    for (auto& contact_map : contacts)
      contact_map.shrinkToFit();

    info.data_storage.setData("contact_results", contacts);
    setData(context, OUTPUT_CONTACT_RESULTS_PORT, contacts, false);

    return info;
  }

  info.color = "green";
  info.status_code = 1;
  info.return_value = 1;
  CONSOLE_BRIDGE_logDebug("%s", info.status_message.c_str());
  return info;
}

}  // namespace tesseract::task_composer

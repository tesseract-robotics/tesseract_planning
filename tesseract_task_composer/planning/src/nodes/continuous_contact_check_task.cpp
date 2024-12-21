/**
 * @file continuous_contact_check_task.cpp
 * @brief Continuous collision check trajectory
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <console_bridge/console.h>

#include <tesseract_common/serialization.h>

#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/serialization.h>

#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

//#include <tesseract_process_managers/core/utils.h>
#include <tesseract_task_composer/planning/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
// Requried
const std::string ContinuousContactCheckTask::INPUT_PROGRAM_PORT = "program";
const std::string ContinuousContactCheckTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string ContinuousContactCheckTask::INPUT_PROFILES_PORT = "profiles";

// Optional
const std::string ContinuousContactCheckTask::INPUT_MANIP_INFO_PORT = "manip_info";
const std::string ContinuousContactCheckTask::OUTPUT_CONTACT_RESULTS_PORT = "contact_results";

ContinuousContactCheckTask::ContinuousContactCheckTask()
  : TaskComposerTask("ContinuousContactCheckTask", ContinuousContactCheckTask::ports(), true)
{
}

ContinuousContactCheckTask::ContinuousContactCheckTask(std::string name,
                                                       std::string input_program_key,
                                                       std::string input_environment_key,
                                                       std::string input_profiles_key,
                                                       bool is_conditional)
  : TaskComposerTask(std::move(name), ContinuousContactCheckTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  validatePorts();
}

ContinuousContactCheckTask::ContinuousContactCheckTask(std::string name,
                                                       const YAML::Node& config,
                                                       const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), ContinuousContactCheckTask::ports(), config)
{
}

TaskComposerNodePorts ContinuousContactCheckTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.input_optional[INPUT_MANIP_INFO_PORT] = TaskComposerNodePorts::SINGLE;
  ports.output_optional[OUTPUT_CONTACT_RESULTS_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

std::unique_ptr<TaskComposerNodeInfo>
ContinuousContactCheckTask::runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor /*executor*/) const
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

  auto input_data_poly = getData(*context.data_storage, INPUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_code = 0;
    info->status_message = "Input seed to ContinuousContactCheckTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    info->return_value = 0;
    return info;
  }

  tesseract_common::ManipulatorInfo input_manip_info;
  auto manip_info_poly = getData(*context.data_storage, INPUT_MANIP_INFO_PORT, false);
  if (!manip_info_poly.isNull())
    input_manip_info = manip_info_poly.as<tesseract_common::ManipulatorInfo>();

  // Get Composite Profile
  auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<ProfileDictionary>>();
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  auto default_profile = std::make_shared<ContactCheckProfile>();
  default_profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  auto cur_composite_profile = getProfile<ContactCheckProfile>(ns_, ci.getProfile(ns_), *profiles, default_profile);

  // Get state solver
  tesseract_common::ManipulatorInfo manip_info = ci.getManipulatorInfo().getCombined(input_manip_info);
  tesseract_kinematics::JointGroup::UPtr manip = env->getJointGroup(manip_info.manipulator);
  tesseract_scene_graph::StateSolver::UPtr state_solver = env->getStateSolver();

  tesseract_collision::ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->applyContactManagerConfig(cur_composite_profile->config.contact_manager_config);

  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, ci, cur_composite_profile->config))
  {
    info->status_code = 0;
    std::stringstream ss;
    ss << "Results are not contact free for process input: " << ci.getDescription() << "\n";
    // Iterate over each contact and add collision summary if not empty
    for (std::size_t i = 0; i < contacts.size(); ++i)
    {
      const auto& contact_map = contacts[i];
      if (!contact_map.empty())
      {
        ss << contact_map.getSummary() << "\n";
      }
    }
    info->status_message = ss.str();
    CONSOLE_BRIDGE_logInform("%s", info->status_message.c_str());

    // Save space
    for (auto& contact_map : contacts)
      contact_map.shrinkToFit();

    info->data_storage.setData("contact_results", contacts);
    setData(*context.data_storage, OUTPUT_CONTACT_RESULTS_PORT, contacts, false);

    info->return_value = 0;
    return info;
  }

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Continuous contact check succeeded";
  CONSOLE_BRIDGE_logDebug("%s", info->status_message.c_str());
  info->return_value = 1;
  return info;
}

bool ContinuousContactCheckTask::operator==(const ContinuousContactCheckTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool ContinuousContactCheckTask::operator!=(const ContinuousContactCheckTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ContinuousContactCheckTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ContinuousContactCheckTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ContinuousContactCheckTask)

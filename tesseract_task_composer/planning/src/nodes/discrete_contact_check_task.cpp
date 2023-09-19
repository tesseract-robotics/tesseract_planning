/**
 * @file discrete_contact_check_task.cpp
 * @brief Discrete collision check trajectory
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
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

//#include <tesseract_process_managers/core/utils.h>
#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_collision/core/serialization.h>

namespace tesseract_planning
{
DiscreteContactCheckTask::DiscreteContactCheckTask() : TaskComposerTask("DiscreteContactCheckTask", true) {}

DiscreteContactCheckTask::DiscreteContactCheckTask(std::string name, std::string input_key, bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
}

DiscreteContactCheckTask::DiscreteContactCheckTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("DiscreteContactCheckTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("DiscreteContactCheckTask, config 'inputs' entry currently only supports one input "
                             "key");
}

TaskComposerNodeInfo::UPtr DiscreteContactCheckTask::runImpl(TaskComposerContext& context,
                                                             OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<DiscreteContactCheckTaskInfo>(*this);
  info->return_value = 0;
  info->env = problem.env;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input to DiscreteContactCheckTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, problem.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<ContactCheckProfile>(name_, profile, *problem.profiles, std::make_shared<ContactCheckProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  // Get state solver
  tesseract_common::ManipulatorInfo manip_info = ci.getManipulatorInfo().getCombined(problem.manip_info);
  tesseract_kinematics::JointGroup::UPtr manip = problem.env->getJointGroup(manip_info.manipulator);
  tesseract_scene_graph::StateSolver::UPtr state_solver = problem.env->getStateSolver();
  tesseract_collision::DiscreteContactManager::Ptr manager = problem.env->getDiscreteContactManager();

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->applyContactManagerConfig(cur_composite_profile->config.contact_manager_config);

  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, ci, cur_composite_profile->config))
  {
    info->message = "Results are not contact free for process input: " + ci.getDescription();
    CONSOLE_BRIDGE_logInform("%s", info->message.c_str());

    // Save space
    for (auto& contact_map : contacts)
      contact_map.shrinkToFit();

    info->contact_results = contacts;
    return info;
  }

  info->color = "green";
  info->message = "Discrete contact check succeeded";
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("%s", info->message.c_str());
  return info;
}

bool DiscreteContactCheckTask::operator==(const DiscreteContactCheckTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool DiscreteContactCheckTask::operator!=(const DiscreteContactCheckTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void DiscreteContactCheckTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

DiscreteContactCheckTaskInfo::DiscreteContactCheckTaskInfo(const DiscreteContactCheckTask& task)
  : TaskComposerNodeInfo(task)
{
}

TaskComposerNodeInfo::UPtr DiscreteContactCheckTaskInfo::clone() const
{
  return std::make_unique<DiscreteContactCheckTaskInfo>(*this);
}

bool DiscreteContactCheckTaskInfo::operator==(const DiscreteContactCheckTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  //  equal &= contact_results == rhs.contact_results;
  return equal;
}
bool DiscreteContactCheckTaskInfo::operator!=(const DiscreteContactCheckTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void DiscreteContactCheckTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
  ar& BOOST_SERIALIZATION_NVP(env);
  ar& BOOST_SERIALIZATION_NVP(contact_results);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DiscreteContactCheckTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DiscreteContactCheckTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DiscreteContactCheckTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DiscreteContactCheckTaskInfo)

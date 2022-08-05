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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

//#include <tesseract_process_managers/core/utils.h>
#include <tesseract_task_composer/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/profiles/contact_check_profile.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
ContinuousContactCheckTask::ContinuousContactCheckTask(std::string input_key, std::string name)
  : TaskComposerNode(std::move(name)), input_key_(std::move(input_key))
{
}

int ContinuousContactCheckTask::run(TaskComposerInput& input) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<ContinuousContactCheckTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage->getData(input_key_);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input seed to ContinuousContactCheckTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<ContactCheckProfile>(name_, profile, *input.profiles, std::make_shared<ContactCheckProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.profile_overrides);

  // Get state solver
  tesseract_kinematics::JointGroup::UPtr manip = input.env->getJointGroup(input.manip_info.manipulator);
  tesseract_scene_graph::StateSolver::UPtr state_solver = input.env->getStateSolver();

  tesseract_collision::ContinuousContactManager::Ptr manager = input.env->getContinuousContactManager();
  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->applyContactManagerConfig(cur_composite_profile->config.contact_manager_config);

  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, ci, cur_composite_profile->config))
  {
    CONSOLE_BRIDGE_logInform("Results are not contact free for process input: %s!", ci.getDescription().c_str());
    for (std::size_t i = 0; i < contacts.size(); i++)
      for (const auto& contact_vec : contacts[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logDebug(("timestep: " + std::to_string(i) + " Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + " Dist: " + std::to_string(contact.distance))
                                      .c_str());
    info->contact_results = contacts;
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Continuous contact check succeeded");
  info->return_value = 1;
  //  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

bool ContinuousContactCheckTask::operator==(const ContinuousContactCheckTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool ContinuousContactCheckTask::operator!=(const ContinuousContactCheckTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ContinuousContactCheckTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

ContinuousContactCheckTaskInfo::ContinuousContactCheckTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr ContinuousContactCheckTaskInfo::clone() const
{
  return std::make_unique<ContinuousContactCheckTaskInfo>(*this);
}

bool ContinuousContactCheckTaskInfo::operator==(const ContinuousContactCheckTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  //  equal &= contact_results == rhs.contact_results;
  return equal;
}
bool ContinuousContactCheckTaskInfo::operator!=(const ContinuousContactCheckTaskInfo& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ContinuousContactCheckTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  //  ar& BOOST_SERIALIZATION_NVP(contact_results);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ContinuousContactCheckTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ContinuousContactCheckTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ContinuousContactCheckTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ContinuousContactCheckTaskInfo)

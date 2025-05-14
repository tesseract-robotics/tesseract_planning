/**
 * @file kinematic_limits_check_taskcpp
 * @brief Kinematic limits check task
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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

#include <tesseract_task_composer/planning/nodes/kinematic_limits_check_task.h>
#include <tesseract_task_composer/planning/profiles/kinematic_limits_check_profile.h>

#include <tesseract_common/serialization.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_kinematics/core/joint_group.h>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_environment/environment.h>

#include <tesseract_task_composer/core/task_composer_context.h>

namespace tesseract_planning
{
// Requried
const std::string KinematicLimitsCheckTask::INPUT_PROGRAM_PORT = "program";
const std::string KinematicLimitsCheckTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string KinematicLimitsCheckTask::INPUT_PROFILES_PORT = "profiles";

KinematicLimitsCheckTask::KinematicLimitsCheckTask()
  : TaskComposerTask("KinematicLimitsCheckTask", KinematicLimitsCheckTask::ports(), true)
{
}

KinematicLimitsCheckTask::KinematicLimitsCheckTask(std::string name,
                                                   std::string input_program_key,
                                                   std::string input_environment_key,
                                                   std::string input_profiles_key,
                                                   bool is_conditional)
  : TaskComposerTask(std::move(name), KinematicLimitsCheckTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  validatePorts();
}

KinematicLimitsCheckTask::KinematicLimitsCheckTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), KinematicLimitsCheckTask::ports(), config)
{
  validatePorts();
}

TaskComposerNodePorts KinematicLimitsCheckTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required["program"] = TaskComposerNodePorts::SINGLE;
  ports.input_required["environment"] = TaskComposerNodePorts::SINGLE;
  ports.input_required["profiles"] = TaskComposerNodePorts::SINGLE;
  return ports;
}

bool KinematicLimitsCheckTask::operator==(const KinematicLimitsCheckTask& rhs) const
{
  return TaskComposerTask::operator==(rhs);
}
bool KinematicLimitsCheckTask::operator!=(const KinematicLimitsCheckTask& rhs) const { return !operator==(rhs); }

TaskComposerNodeInfo KinematicLimitsCheckTask::runImpl(TaskComposerContext& context,
                                                       OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  if (context.isAborted())
  {
    info.status_message = "Aborted";
    return info;
  }

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
  {
    info.color = "red";
    info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    return info;
  }

  auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

  auto input_data_poly = getData(*context.data_storage, INPUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info.color = "red";
    info.status_message = "Input to KinematicLimitsCheckTask must be a composite instruction";
    return info;
  }

  // Get Composite Profile
  auto profiles =
      getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<tesseract_common::ProfileDictionary>>();
  auto& ci = input_data_poly.as<CompositeInstruction>();
  auto cur_composite_profile = getProfile<KinematicLimitsCheckProfile>(
      ns_, ci.getProfile(ns_), *profiles, std::make_shared<KinematicLimitsCheckProfile>());

  const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

  // Create data structures for checking for plan profile overrides
  auto flattened = ci.flatten(moveFilter);
  if (flattened.empty())
  {
    info.color = "yellow";
    info.status_message = "Kinematic limits check found no MoveInstructions to process";
    info.status_code = 1;
    info.return_value = 1;
    return info;
  }

  // Wrap the composite instruction in a trajectory container
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(flattened);

  // Extract the motion group
  tesseract_kinematics::JointGroup::ConstPtr motion_group = env->getJointGroup(manip_info.manipulator);
  const auto limits = motion_group->getLimits();

  // Check the trajectory limits
  for (Eigen::Index i = 0; i < trajectory->size(); ++i)
  {
    const Eigen::VectorXd& joint_pos = trajectory->getPosition(i);
    const Eigen::VectorXd& joint_vel = trajectory->getVelocity(i);
    const Eigen::VectorXd& joint_acc = trajectory->getAcceleration(i);

    if (cur_composite_profile->check_position)
    {
      if (!tesseract_common::satisfiesLimits<double>(joint_pos, limits.joint_limits))
      {
        std::stringstream ss;
        ss << "Joint position limit violation(s) at waypoint " << i;
        info.color = "red";
        info.status_message = ss.str();
        return info;
      }
    }

    if (cur_composite_profile->check_velocity)
    {
      // Check for joint velocity limit violations
      if (!tesseract_common::satisfiesLimits<double>(joint_vel, limits.velocity_limits))
      {
        Eigen::ArrayXd capacity = 100.0 * joint_vel.array().abs() / limits.velocity_limits.col(1).array();
        std::stringstream ss;
        ss << "Joint velocity limit violation(s) at waypoint " << i << ": "
           << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";

        info.color = "red";
        info.status_message = ss.str();
        return info;
      }
    }

    if (cur_composite_profile->check_acceleration)
    {
      // Check for joint velocity acceleration limit violations
      if (!tesseract_common::satisfiesLimits<double>(joint_acc, limits.acceleration_limits))
      {
        Eigen::ArrayXd capacity = 100.0 * joint_acc.array().abs() / limits.acceleration_limits.col(1).array();
        std::stringstream ss;
        ss << "Joint acceleration limit violation(s) at waypoint " << i << ": "
           << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";

        info.color = "red";
        info.status_message = ss.str();
        return info;
      }
    }
  }

  info.color = "green";
  info.status_message = "Kinematic limits check succeeded";
  info.status_code = 1;
  info.return_value = 1;
  return info;
}

template <class Archive>
void KinematicLimitsCheckTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::KinematicLimitsCheckTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::KinematicLimitsCheckTask)

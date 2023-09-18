/**
 * @file fix_state_collision_task.h
 * @brief Task that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_TASK_H
#define TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_command_language/poly/waypoint_poly.h>

#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
/**
 * @brief This task modifies the input instructions in order to push waypoints that are in collision out of
 * collision.
 *
 * First it uses TrajOpt to correct the waypoint. If that fails, it reverts to random sampling
 */
class FixStateCollisionTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<FixStateCollisionTask>;
  using ConstPtr = std::shared_ptr<const FixStateCollisionTask>;
  using UPtr = std::unique_ptr<FixStateCollisionTask>;
  using ConstUPtr = std::unique_ptr<const FixStateCollisionTask>;

  FixStateCollisionTask();
  explicit FixStateCollisionTask(std::string name,
                                 std::string input_key,
                                 std::string output_key,
                                 bool conditional = true);
  explicit FixStateCollisionTask(std::string name,
                                 const YAML::Node& config,
                                 const TaskComposerPluginFactory& plugin_factory);
  ~FixStateCollisionTask() override = default;
  FixStateCollisionTask(const FixStateCollisionTask&) = delete;
  FixStateCollisionTask& operator=(const FixStateCollisionTask&) = delete;
  FixStateCollisionTask(FixStateCollisionTask&&) = delete;
  FixStateCollisionTask& operator=(FixStateCollisionTask&&) = delete;

  bool operator==(const FixStateCollisionTask& rhs) const;
  bool operator!=(const FixStateCollisionTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

class FixStateCollisionTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<FixStateCollisionTaskInfo>;
  using ConstPtr = std::shared_ptr<const FixStateCollisionTaskInfo>;
  using UPtr = std::unique_ptr<FixStateCollisionTaskInfo>;
  using ConstUPtr = std::unique_ptr<const FixStateCollisionTaskInfo>;

  FixStateCollisionTaskInfo() = default;
  FixStateCollisionTaskInfo(const FixStateCollisionTask& task);

  TaskComposerNodeInfo::UPtr clone() const override;

  tesseract_environment::Environment::ConstPtr env;
  std::vector<tesseract_collision::ContactResultMap> contact_results;

  bool operator==(const FixStateCollisionTaskInfo& rhs) const;
  bool operator!=(const FixStateCollisionTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief Checks if a joint state is in collision
 * @param start_pos Vector that represents a joint state
 * @param problem Process problem associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool stateInCollision(const Eigen::Ref<const Eigen::VectorXd>& start_pos,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const PlanningTaskComposerProblem& problem,
                      const FixStateCollisionProfile& profile,
                      tesseract_collision::ContactResultMap& contacts);

/**
 * @brief Checks if a waypoint is in collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param problem Process problem associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool waypointInCollision(const WaypointPoly& waypoint,
                         const tesseract_common::ManipulatorInfo& manip_info,
                         const PlanningTaskComposerProblem& problem,
                         const FixStateCollisionProfile& profile,
                         tesseract_collision::ContactResultMap& contacts);

/**
 * @brief Takes a waypoint and uses a small trajopt problem to push it out of collision if necessary
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param problem Process problem associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool moveWaypointFromCollisionTrajopt(WaypointPoly& waypoint,
                                      const tesseract_common::ManipulatorInfo& manip_info,
                                      const PlanningTaskComposerProblem& problem,
                                      const FixStateCollisionProfile& profile);

/**
 * @brief Takes a waypoint and uses random sampling to find a position that is out of collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param problem Process problem associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool moveWaypointFromCollisionRandomSampler(WaypointPoly& waypoint,
                                            const tesseract_common::ManipulatorInfo& manip_info,
                                            const PlanningTaskComposerProblem& problem,
                                            const FixStateCollisionProfile& profile);

bool applyCorrectionWorkflow(WaypointPoly& waypoint,
                             const tesseract_common::ManipulatorInfo& manip_info,
                             const PlanningTaskComposerProblem& problem,
                             const FixStateCollisionProfile& profile);
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateCollisionTask, "FixStateCollisionTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateCollisionTaskInfo, "FixStateCollisionTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_TASK_H

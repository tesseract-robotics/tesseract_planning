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

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>
#include <tesseract_task_composer/profiles/fix_state_collision_profile.h>

namespace tesseract_planning
{
/**
 * @brief This task modifies the const input instructions in order to push waypoints that are in collision out of
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

  FixStateCollisionTask() = default;  // Required for serialization
  FixStateCollisionTask(std::string input_key,
                        std::string output_key,
                        bool is_conditional = true,
                        std::string name = profile_ns::FIX_STATE_COLLISION_DEFAULT_NAMESPACE);

  ~FixStateCollisionTask() override = default;
  FixStateCollisionTask(const FixStateCollisionTask&) = delete;
  FixStateCollisionTask& operator=(const FixStateCollisionTask&) = delete;
  FixStateCollisionTask(FixStateCollisionTask&&) = delete;
  FixStateCollisionTask& operator=(FixStateCollisionTask&&) = delete;

  int run(TaskComposerInput& input) const override final;

  bool operator==(const FixStateCollisionTask& rhs) const;
  bool operator!=(const FixStateCollisionTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string input_key_;
  std::string output_key_;
};

class FixStateCollisionTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<FixStateCollisionTaskInfo>;
  using ConstPtr = std::shared_ptr<const FixStateCollisionTaskInfo>;
  using UPtr = std::unique_ptr<FixStateCollisionTaskInfo>;
  using ConstUPtr = std::unique_ptr<const FixStateCollisionTaskInfo>;

  FixStateCollisionTaskInfo() = default;
  FixStateCollisionTaskInfo(boost::uuids::uuid uuid,
                            std::string name = profile_ns::FIX_STATE_COLLISION_DEFAULT_NAMESPACE);

  std::vector<tesseract_collision::ContactResultMap> contact_results;

  TaskComposerNodeInfo::UPtr clone() const override;

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
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool stateInCollision(const Eigen::Ref<const Eigen::VectorXd>& start_pos,
                      const TaskComposerInput& input,
                      const FixStateCollisionProfile& profile,
                      tesseract_collision::ContactResultMap& contacts);

/**
 * @brief Checks if a waypoint is in collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @return True if in collision
 */
bool waypointInCollision(const WaypointPoly& waypoint,
                         const TaskComposerInput& input,
                         const FixStateCollisionProfile& profile,
                         tesseract_collision::ContactResultMap& contacts);

/**
 * @brief Takes a waypoint and uses a small trajopt problem to push it out of collision if necessary
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool moveWaypointFromCollisionTrajopt(WaypointPoly& waypoint,
                                      const TaskComposerInput& input,
                                      const FixStateCollisionProfile& profile);

/**
 * @brief Takes a waypoint and uses random sampling to find a position that is out of collision
 * @param waypoint Must be a waypoint for which getJointPosition will return a position
 * @param input Process Input associated with waypoint. Needed for kinematics, etc.
 * @param profile Profile containing needed params
 * @return True if successful
 */
bool moveWaypointFromCollisionRandomSampler(WaypointPoly& waypoint,
                                            const TaskComposerInput& input,
                                            const FixStateCollisionProfile& profile);

bool applyCorrectionWorkflow(WaypointPoly& waypoint,
                             const TaskComposerInput& input,
                             const FixStateCollisionProfile& profile);
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateCollisionTask, "FixStateCollisionTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateCollisionTaskInfo, "FixStateCollisionTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_TASK_H

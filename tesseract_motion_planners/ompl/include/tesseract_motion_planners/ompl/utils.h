/**
 * @file utils.h
 * @brief Tesseract OMPL planner utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <functional>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_types.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_command_language/composite_instruction.h>

namespace ompl::base
{
class StateSpace;
using StateSpacePtr = std::shared_ptr<StateSpace>;
class StateSampler;
using StateSamplerPtr = std::shared_ptr<StateSampler>;
}  // namespace ompl::base

namespace ompl::geometric
{
class PathGeometric;
}

namespace tesseract_planning
{
/**
 * @brief Given longest valid fraction and length it will set the correct information of the state space
 * @param state_space_ptr OMPL State Space
 * @param longest_valid_segment_fraction
 * @param longest_valid_segment_length
 */
void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                double longest_valid_segment_fraction,
                                double longest_valid_segment_length);

/**
 * @brief Given collision check config set ompl longest_valid_segment_fraction
 * @param state_space_ptr OMPL State Space
 * @param collision_check_config
 */
void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                const tesseract_collision::CollisionCheckConfig& collision_check_config);

/**
 * @brief For the provided problem check if the state is in collision
 * @param contact_map Map of contact results. Will be empty if return true
 * @param contact_checker The contact checker to leverage
 * @param manip The manipulator for calculating forward kinematics
 * @param state The joint state
 * @return True if in collision otherwise false
 */
bool checkStateInCollision(tesseract_collision::ContactResultMap& contact_map,
                           tesseract_collision::DiscreteContactManager& contact_checker,
                           const tesseract_kinematics::JointGroup& manip,
                           const Eigen::VectorXd& state);

// long assignTrajectory(tesseract_planning::CompositeInstruction& output,
//                       boost::uuids::uuid start_uuid,
//                       boost::uuids::uuid end_uuid,
//                       long start_index,
//                       const std::vector<std::string>& joint_names,
//                       const tesseract_common::TrajArray& traj,
//                       const bool format_result_as_input);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H

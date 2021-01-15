/**
 * @file simple_planner_lvs_plan_profile.h
 * @brief
 *
 * @author Tyler Marr
 * @date September 16, 2020
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

#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_LVS_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_LVS_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::SimplePlannerLVSPlanProfile)
#endif  // SWIG

namespace tesseract_planning
{
class SimplePlannerLVSPlanProfile : public SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerLVSPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerLVSPlanProfile>;

  /**
   * @brief SimplePlannerLVSPlanProfile
   * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions)
   *between successive steps
   * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
   * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
   * @param min_steps The minimum number of steps for the plan
   */
  SimplePlannerLVSPlanProfile(double state_longest_valid_segment_length = 5 * M_PI / 180,
                              double translation_longest_valid_segment_length = 0.1,
                              double rotation_longest_valid_segment_length = 5 * M_PI / 180,
                              int min_steps = 1);

  CompositeInstruction generate(const PlanInstruction& prev_instruction,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& global_manip_info) const override;

  /** @brief The maximum joint distance, the norm of changes to all joint positions between successive steps. */
  double state_longest_valid_segment_length;

  /** @brief The maximum translational distance between successive steps */
  double translation_longest_valid_segment_length;

  /** @brief The maximaum rotational distance between successive steps */
  double rotation_longest_valid_segment_length;

  /** @brief The minimum number of steps for the plan */
  int min_steps;

protected:
  /**
   * @brief JointWaypoint to JointWaypoint
   *
   * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
   *
   * - the number of steps for the plan will be calculated such that:
   *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
   *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
   *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
   *successive steps is no longer than state_longest_valid_segment_length
   *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
   * - the interpolation will be done in joint space
   *
   * @return A composite instruction of move instruction with state waypoints
   **/
  CompositeInstruction stateJointJointWaypoint(const InstructionInfo& prev, const InstructionInfo& base) const;

  /**
   * @brief JointWaypoint to CartesianWaypoint
   *
   * - the number of steps for the plan will be calculated such that:
   *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
   *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
   *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
   *successive steps is no longer than state_longest_valid_segment_length
   *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
   * - the interpolation will be done based on the condition below
   *   - Case 1: Joint solution found for end cartesian waypoint
   *     - It interpolates the joint position from the start to the end state
   *   - Case 2: Unable to find joint solution for end cartesian waypoint
   *     - It creates number states based on the steps and sets the value to start joint waypoint
   *
   * @return A composite instruction of move instruction with state waypoints
   **/
  CompositeInstruction stateJointCartWaypoint(const InstructionInfo& prev, const InstructionInfo& base) const;

  /**
   * @brief CartesianWaypoint to JointWaypoint
   *
   * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
   *
   * - the number of steps for the plan will be calculated such that:
   *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
   *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
   *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
   *successive steps is no longer than state_longest_valid_segment_length
   *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
   * - the interpolation will be done based on the condition below
   *   - Case 1: Joint solution found for start cartesian waypoint
   *     - It interpolates the joint position from the start to the end state
   *   - Case 2: Unable to find joint solution for start cartesian waypoint
   *     - It creates number states based on the steps and sets the value to end joint waypoint
   *
   * @return A composite instruction of move instruction with state waypoints
   **/
  CompositeInstruction stateCartJointWaypoint(const InstructionInfo& prev, const InstructionInfo& base) const;

  /**
   * @brief CartesianWaypoint to CartesianWaypoint
   *
   * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
   *
   * - the number of steps for the plan will be calculated such that:
   *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
   *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
   *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
   *successive steps is no longer than state_longest_valid_segment_length
   *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
   * - the interpolation will be done based on the condition below
   *   - Case 1: Joint solution found for start and end cartesian waypoint
   *     - It interpolates the joint position from the start to the end state
   *   - Case 2: Joint solution only found for start cartesian waypoint
   *     - It creates number states based on the steps and sets the value to found start solution
   *   - Case 3: Joint solution only found for end cartesian waypoint
   *     - It creates number states based on the steps and sets the value to found end solution
   *   - Case 4: No joint solution found for end and start cartesian waypoint
   *     - It creates number states based on the steps and sets the value to the current state of the environment
   *
   * @return A composite instruction of move instruction with state waypoints
   **/
  CompositeInstruction stateCartCartWaypoint(const InstructionInfo& prev,
                                             const InstructionInfo& base,
                                             const PlannerRequest& request) const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_LVS_PLAN_PROFILE_H

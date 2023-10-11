/**
 * @file planner.h
 * @brief Planner Interface Class.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_H

#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
class MotionPlanner
{
public:
  using Ptr = std::shared_ptr<MotionPlanner>;
  using ConstPtr = std::shared_ptr<const MotionPlanner>;
  /** @brief Construct a basic planner */
  MotionPlanner() = default;
  MotionPlanner(std::string name);
  virtual ~MotionPlanner() = default;
  MotionPlanner(const MotionPlanner&) = delete;
  MotionPlanner& operator=(const MotionPlanner&) = delete;
  MotionPlanner(MotionPlanner&&) = delete;
  MotionPlanner& operator=(MotionPlanner&&) = delete;

  /**
   *  @brief Get the name of this planner
   *  @details This is also used as the namespace for the profiles in the profile dictionary
   */
  const std::string& getName() const;

  /**
   * @brief Solve the planner request problem
   * @param request The planning request
   * @return A planner reponse
   */
  virtual PlannerResponse solve(const PlannerRequest& request) const = 0;

  /**
   * @brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).
   */
  virtual bool terminate() = 0;

  /** @brief Clear the data structures used by the planner */
  virtual void clear() = 0;

  /** @brief Clone the motion planner */
  virtual MotionPlanner::Ptr clone() const = 0;

  /** @brief Check planning request */
  static bool checkRequest(const PlannerRequest& request);

  /** @brief Assign a solution to the move instruction */
  static void assignSolution(MoveInstructionPoly& mi,
                             const std::vector<std::string>& joint_names,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                             bool format_result_as_input);

protected:
  std::string name_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_PLANNER_H

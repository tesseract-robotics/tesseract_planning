/**
 * @file online_planning_example.h
 * @brief This example demonstrates using trajopt to plan in an "online" manner. As the environment is changed (using
 * the joint state publisher), the system will attempt to avoid collisions with the moving object and follow the moving
 * target
 *
 * Note: If the target moves too quickly the solver can get stuck in an infeasible point. That is the nature of how the
 * solver is working. Higher level intelligence, a larger step size, or changing the target to a cost can solve that
 * problem.
 *
 * @author Matthew Powelson
 * @date June 9, 2020
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
#ifndef TTESSERACT_EXAMPLES_ONLINE_PLANNING_EXAMPLE_H
#define TTESSERACT_EXAMPLES_ONLINE_PLANNING_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <Eigen/Core>
#include <trajopt_sqp/qp_problem.h>

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/example.h>
#include <tesseract_visualization/trajectory_player.h>

namespace tesseract_examples
{
/**
 * @brief This example demonstrates running the inner loop of TrajOpt in an "online" manner. As the environment changes,
 * TrajOpt dynamically adjusts to avoid collisions and follow the given toolpath
 */
class OnlinePlanningExample : public Example
{
public:
  OnlinePlanningExample(tesseract_environment::Environment::Ptr env,
                        tesseract_visualization::Visualization::Ptr plotter = nullptr,
                        int steps = 12,
                        double box_size = 0.01,
                        bool update_start_state = false,
                        bool use_continuous = false);

  ~OnlinePlanningExample() override = default;
  OnlinePlanningExample(const OnlinePlanningExample&) = delete;
  OnlinePlanningExample& operator=(const OnlinePlanningExample&) = delete;
  OnlinePlanningExample(OnlinePlanningExample&&) = default;
  OnlinePlanningExample& operator=(OnlinePlanningExample&&) = default;

  /**
   * @brief Runs the example
   * @return True if successful
   */
  bool run() override;

  /**
   * @brief Update the current state
   * @param joint_names The joint names
   * @param joint_values The joint values
   */
  void updateState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);

  /**
   * @brief Toggle real-time mode
   * @param enable True/False
   */
  void toggleRealtime(bool enable);

  /**
   * @brief Sets up the IFOPT problem
   * @return True if successful
   */
  bool setupProblem(const std::vector<Eigen::VectorXd>& initial_trajectory = std::vector<Eigen::VectorXd>());

  /**
   * @brief Runs the online planning process, occasionally checking ROS callbacks until realtime_running_ = false
   * @return True if successful
   */
  bool onlinePlan();

private:
  int steps_;
  double box_size_;
  bool update_start_state_{ false };
  bool use_continuous_{ false };
  bool realtime_running_;

  tesseract_kinematics::KinematicGroup::ConstPtr manip_;
  tesseract_visualization::TrajectoryPlayer player_;
  tesseract_common::TrajArray current_trajectory_;
  Eigen::Isometry3d target_pose_delta_;
  Eigen::Isometry3d target_pose_base_frame_;

  // We need to keep this around so we can update it
  trajopt_ifopt::CartPosConstraint::Ptr target_pose_constraint_;
  std::vector<std::string> joint_names_;
  trajopt_sqp::QPProblem::Ptr nlp_;

  void updateAndPlotTrajectory(const Eigen::VectorXd& osqp_vals);
};
}  // namespace tesseract_examples

#endif  // TTESSERACT_EXAMPLES_ONLINE_PLANNING_EXAMPLE_H

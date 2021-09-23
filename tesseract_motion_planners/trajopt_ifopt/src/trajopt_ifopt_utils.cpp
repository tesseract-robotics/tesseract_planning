/**
 * @file trajopt_ifopt_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_command_language/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_collision/core/common.h>

#include <ifopt/problem.h>

namespace tesseract_planning
{
ifopt::ConstraintSet::Ptr createCartesianPositionConstraint(const Eigen::Isometry3d& target,
                                                            const trajopt_ifopt::JointPosition::ConstPtr& var,
                                                            const trajopt_ifopt::KinematicsInfo::ConstPtr& kin_info,
                                                            const std::string& source_link,
                                                            const Eigen::Isometry3d& source_tcp,
                                                            const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<int> indices;
  std::vector<double> constraint_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)
  {
    if (!tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      indices.push_back(static_cast<int>(i));
      constraint_coeffs.push_back(coeffs(i));
    }
  }

  trajopt_ifopt::CartPosInfo cart_info(
      kin_info,
      target,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXi>(indices.data(), static_cast<Eigen::Index>(indices.size())));
  auto constraint = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, var, "CartPos_" + var->GetName());
  return constraint;
}

bool addCartesianPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                    const Eigen::Isometry3d& target,
                                    const trajopt_ifopt::JointPosition::ConstPtr& var,
                                    const trajopt_ifopt::KinematicsInfo::ConstPtr& kin_info,
                                    const std::string& source_link,
                                    const Eigen::Isometry3d& source_tcp,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  auto constraint = createCartesianPositionConstraint(target, var, kin_info, source_link, source_tcp, coeffs);
  nlp.addConstraintSet(constraint);
  return true;
}

bool addCartesianPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const Eigen::Isometry3d& target,
                                     const trajopt_ifopt::JointPosition::ConstPtr& var,
                                     const trajopt_ifopt::KinematicsInfo::ConstPtr& kin_info,
                                     const std::string& source_link,
                                     const Eigen::Isometry3d& source_tcp,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<double> constraint_coeffs;
  std::vector<double> cost_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)
  {
    if (tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      constraint_coeffs.push_back(0);
    }
    else
    {
      constraint_coeffs.push_back(1);
      cost_coeffs.push_back(coeffs(i));
    }
  }

  auto constraint = createCartesianPositionConstraint(
      target,
      var,
      kin_info,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

bool addCartesianPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                      const Eigen::Isometry3d& target,
                                      const trajopt_ifopt::JointPosition::ConstPtr& var,
                                      const trajopt_ifopt::KinematicsInfo::ConstPtr& kin_info,
                                      const std::string& source_link,
                                      const Eigen::Isometry3d& source_tcp,
                                      const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<double> constraint_coeffs;
  std::vector<double> cost_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)  // NOLINT
  {
    if (tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      constraint_coeffs.push_back(0);
    }
    else
    {
      constraint_coeffs.push_back(1);
      cost_coeffs.push_back(coeffs(i));
    }
  }

  auto constraint = createCartesianPositionConstraint(
      target,
      var,
      kin_info,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::ABSOLUTE);
  return true;
}

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypoint& joint_waypoint,
                                                        const trajopt_ifopt::JointPosition::ConstPtr& var,
                                                        const Eigen::VectorXd& coeffs)
{
  assert(var);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars(1, var);

  ifopt::ConstraintSet::Ptr constraint;
  if (!joint_waypoint.isToleranced())
  {
    constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(
        joint_waypoint.waypoint, vars, coeffs, "JointPos_" + var->GetName());
  }
  else
  {
    Eigen::VectorXd lower_limit = joint_waypoint.waypoint + joint_waypoint.lower_tolerance;
    Eigen::VectorXd upper_limit = joint_waypoint.waypoint + joint_waypoint.upper_tolerance;
    auto bounds = trajopt_ifopt::toBounds(lower_limit, upper_limit);
    constraint =
        std::make_shared<trajopt_ifopt::JointPosConstraint>(bounds, vars, coeffs, "JointPos_" + var->GetName());
  }

  return constraint;
}

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                           const tesseract_environment::Environment::ConstPtr& env,
                           const ManipulatorInfo& manip_info,
                           const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                           const std::vector<int>& fixed_indices)
{
  std::vector<ifopt::ConstraintSet::Ptr> constraints;
  if (config->type == tesseract_collision::CollisionEvaluatorType::NONE)
    return constraints;

  // Add a collision cost for all steps
  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(vars.size());
  if (config->type == tesseract_collision::CollisionEvaluatorType::DISCRETE)
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      auto active_link_names = env->getActiveLinkNames(kin->getJointNames());
      auto static_link_names = env->getStaticLinkNames(kin->getJointNames());
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      constraints.push_back(std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
          collision_evaluator,
          vars[i],
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          "DiscreteCollision_" + std::to_string(i)));
    }
  }
  else if (config->type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    bool time0_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), 0) != fixed_indices.end());
    for (std::size_t i = 1; i < vars.size(); ++i)
    {
      bool time1_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end());

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSDiscreteCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      auto active_link_names = env->getActiveLinkNames(kin->getJointNames());
      auto static_link_names = env->getStaticLinkNames(kin->getJointNames());
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
      std::array<bool, 2> position_vars_fixed{ time0_fixed, time1_fixed };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator,
          position_vars,
          position_vars_fixed,
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          "LVSDiscreteCollision_" + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }
  else
  {
    bool time0_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), 0) != fixed_indices.end());
    for (std::size_t i = 1; i < vars.size(); ++i)
    {
      bool time1_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end());

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      auto active_link_names = env->getActiveLinkNames(kin->getJointNames());
      auto static_link_names = env->getStaticLinkNames(kin->getJointNames());
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
      std::array<bool, 2> position_vars_fixed{ time0_fixed, time1_fixed };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator,
          position_vars,
          position_vars_fixed,
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          "LVSDiscreteCollision_" + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }

  return constraints;
}

bool addCollisionConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                            const tesseract_environment::Environment::ConstPtr& env,
                            const ManipulatorInfo& manip_info,
                            const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                            const std::vector<int>& fixed_indices)
{
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices);
  for (auto& constraint : constraints)
    nlp.addConstraintSet(constraint);
  return true;
}

bool addCollisionCost(trajopt_sqp::QPProblem& nlp,
                      const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                      const tesseract_environment::Environment::ConstPtr& env,
                      const ManipulatorInfo& manip_info,
                      const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                      const std::vector<int>& fixed_indices)
{
  // Coefficients are applied within the constraint
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices);
  for (auto& constraint : constraints)
    nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::HINGE);

  return true;
}

ifopt::ConstraintSet::Ptr createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                                        const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                                        const Eigen::VectorXd& /*coeffs*/)

{
  assert(!vars.empty());
  trajopt_ifopt::JointVelConstraint::Ptr vel_constraint =
      std::make_shared<trajopt_ifopt::JointVelConstraint>(target, vars, "JointVelocity");
  return vel_constraint;
}

bool addJointVelocityConstraint(trajopt_sqp::QPProblem& nlp,
                                const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);
  nlp.addConstraintSet(vel_constraint);
  return true;
}

bool addJointVelocitySquaredCost(trajopt_sqp::QPProblem& nlp,
                                 const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);
  nlp.addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

}  // namespace tesseract_planning

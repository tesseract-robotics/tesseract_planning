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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_sqp/qp_problem.h>
#include <ifopt/constraint_set.h>
#include <OsqpEigen/Settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>

#include <tesseract_common/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>

namespace tesseract_planning
{
void copyOSQPEigenSettings(OsqpEigen::Settings& lhs, const OsqpEigen::Settings& rhs)
{
  const auto& settings = *rhs.getSettings();
  lhs.setRho(settings.rho);
  lhs.setSigma(settings.sigma);
  lhs.setScaling(static_cast<int>(settings.scaling));
  lhs.setAdaptiveRho(static_cast<bool>(settings.adaptive_rho));
  lhs.setAdaptiveRhoInterval(static_cast<int>(settings.adaptive_rho_interval));
  lhs.setAdaptiveRhoTolerance(settings.adaptive_rho_tolerance);
  lhs.setAdaptiveRhoFraction(settings.adaptive_rho_fraction);
  lhs.setMaxIteration(static_cast<int>(settings.max_iter));
  lhs.setAbsoluteTolerance(settings.eps_abs);
  lhs.setRelativeTolerance(settings.eps_rel);
  lhs.setPrimalInfeasibilityTollerance(settings.eps_prim_inf);
  lhs.setDualInfeasibilityTollerance(settings.eps_dual_inf);
  lhs.setAlpha(settings.alpha);
  lhs.setLinearSystemSolver(settings.linsys_solver);
  lhs.setDelta(settings.delta);
  lhs.setPolish(static_cast<bool>(settings.polish));
  lhs.setPolishRefineIter(static_cast<int>(settings.polish_refine_iter));
  lhs.setVerbosity(static_cast<bool>(settings.verbose));
  lhs.setScaledTerimination(static_cast<bool>(settings.scaled_termination));
  lhs.setCheckTermination(static_cast<int>(settings.check_termination));
  lhs.setWarmStart(static_cast<bool>(settings.warm_start));
  lhs.setTimeLimit(settings.time_limit);
}

ifopt::ConstraintSet::Ptr
createCartesianPositionConstraint(const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                  const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                  const std::string& source_frame,
                                  const std::string& target_frame,
                                  const Eigen::Isometry3d& source_frame_offset,
                                  const Eigen::Isometry3d& target_frame_offset,
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
      manip,
      source_frame,
      target_frame,
      source_frame_offset,
      target_frame_offset,
      Eigen::Map<Eigen::VectorXi>(indices.data(), static_cast<Eigen::Index>(indices.size())));
  auto constraint = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, var, "CartPos_" + var->GetName());
  return constraint;
}

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                           const std::shared_ptr<const tesseract_environment::Environment>& env,
                           const tesseract_common::ManipulatorInfo& manip_info,
                           const std::shared_ptr<const trajopt_common::TrajOptCollisionConfig>& config,
                           const std::vector<int>& fixed_indices,
                           bool fixed_sparsity)
{
  std::vector<ifopt::ConstraintSet::Ptr> constraints;
  if (config->type == tesseract_collision::CollisionEvaluatorType::NONE)
    return constraints;

  // Add a collision cost for all steps
  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(vars.size());
  std::unordered_map<std::string, std::shared_ptr<const tesseract_kinematics::JointGroup>> manipulators;
  if (config->type == tesseract_collision::CollisionEvaluatorType::DISCRETE)
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      std::shared_ptr<const tesseract_kinematics::JointGroup> manip;
      auto it = manipulators.find(manip_info.manipulator);
      if (it != manipulators.end())
      {
        manip = it->second;
      }
      else
      {
        manip = env->getJointGroup(manip_info.manipulator);
        manipulators[manip_info.manipulator] = manip;
      }

      auto collision_evaluator =
          std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(collision_cache, manip, env, config);

      auto active_link_names = manip->getActiveLinkNames();
      auto static_link_names = manip->getStaticLinkNames();
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      constraints.push_back(std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
          collision_evaluator,
          vars[i],
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          fixed_sparsity,
          "DiscreteCollision_" + std::to_string(i)));
    }
  }
  else if (config->type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    bool time0_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), 0) != fixed_indices.end());
    for (std::size_t i = 1; i < vars.size(); ++i)
    {
      bool time1_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end());

      std::shared_ptr<const tesseract_kinematics::JointGroup> manip;
      auto it = manipulators.find(manip_info.manipulator);
      if (it != manipulators.end())
      {
        manip = it->second;
      }
      else
      {
        manip = env->getJointGroup(manip_info.manipulator);
        manipulators[manip_info.manipulator] = manip;
      }

      auto collision_evaluator =
          std::make_shared<trajopt_ifopt::LVSDiscreteCollisionEvaluator>(collision_cache, manip, env, config);

      auto active_link_names = manip->getActiveLinkNames();
      auto static_link_names = manip->getStaticLinkNames();
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
      std::array<bool, 2> position_vars_fixed{ time0_fixed, time1_fixed };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator,
          position_vars,
          position_vars_fixed,
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          fixed_sparsity,
          "LVSDiscreteCollision_" + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }
  else
  {
    const std::string prefix = (config->type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS) ? "LVSCont"
                                                                                                               "inuousC"
                                                                                                               "ollisio"
                                                                                                               "n_" :
                                                                                                               "Continu"
                                                                                                               "ousColl"
                                                                                                               "ision_";
    bool time0_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), 0) != fixed_indices.end());
    for (std::size_t i = 1; i < vars.size(); ++i)
    {
      bool time1_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end());

      std::shared_ptr<const tesseract_kinematics::JointGroup> manip;
      auto it = manipulators.find(manip_info.manipulator);
      if (it != manipulators.end())
      {
        manip = it->second;
      }
      else
      {
        manip = env->getJointGroup(manip_info.manipulator);
        manipulators[manip_info.manipulator] = manip;
      }

      auto collision_evaluator =
          std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(collision_cache, manip, env, config);

      auto active_link_names = manip->getActiveLinkNames();
      auto static_link_names = manip->getStaticLinkNames();
      auto cp = tesseract_collision::getCollisionObjectPairs(
          active_link_names, static_link_names, env->getDiscreteContactManager()->getIsContactAllowedFn());

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
      std::array<bool, 2> position_vars_fixed{ time0_fixed, time1_fixed };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator,
          position_vars,
          position_vars_fixed,
          std::min(config->max_num_cnt, static_cast<int>(cp.size())),
          fixed_sparsity,
          prefix + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }

  return constraints;
}

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypointPoly& joint_waypoint,
                                                        const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                                        const Eigen::VectorXd& coeffs)
{
  assert(var);
  std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>> vars(1, var);

  ifopt::ConstraintSet::Ptr constraint;
  if (!joint_waypoint.isToleranced())
  {
    constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(
        joint_waypoint.getPosition(), vars, coeffs, "JointPos_" + var->GetName());
  }
  else
  {
    Eigen::VectorXd lower_limit = joint_waypoint.getPosition() + joint_waypoint.getLowerTolerance();
    Eigen::VectorXd upper_limit = joint_waypoint.getPosition() + joint_waypoint.getUpperTolerance();
    auto bounds = trajopt_ifopt::toBounds(lower_limit, upper_limit);
    constraint =
        std::make_shared<trajopt_ifopt::JointPosConstraint>(bounds, vars, coeffs, "JointPos_" + var->GetName());
  }

  return constraint;
}

ifopt::ConstraintSet::Ptr
createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                              const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                              const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(target, vars, coeffs, "JointVelocity");
  return vel_constraint;
}

ifopt::ConstraintSet::Ptr
createJointAccelerationConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                  const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                  const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto accel_constraint =
      std::make_shared<trajopt_ifopt::JointAccelConstraint>(target, vars, coeffs, "JointAcceleration");
  return accel_constraint;
}

ifopt::ConstraintSet::Ptr
createJointJerkConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                          const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                          const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto jerk_constraint = std::make_shared<trajopt_ifopt::JointJerkConstraint>(target, vars, coeffs, "JointJerk");
  return jerk_constraint;
}

bool addCartesianPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                    const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                    const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                    const std::string& source_frame,
                                    const std::string& target_frame,
                                    const Eigen::Isometry3d& source_frame_offset,
                                    const Eigen::Isometry3d& target_frame_offset,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  auto constraint = createCartesianPositionConstraint(
      var, manip, source_frame, target_frame, source_frame_offset, target_frame_offset, coeffs);
  nlp.addConstraintSet(constraint);
  return true;
}

bool addCartesianPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                     const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                     const std::string& source_frame,
                                     const std::string& target_frame,
                                     const Eigen::Isometry3d& source_frame_offset,
                                     const Eigen::Isometry3d& target_frame_offset,
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
      var,
      manip,
      source_frame,
      target_frame,
      source_frame_offset,
      target_frame_offset,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

bool addCartesianPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                      const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                      const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                      const std::string& source_frame,
                                      const std::string& target_frame,
                                      const Eigen::Isometry3d& source_frame_offset,
                                      const Eigen::Isometry3d& target_frame_offset,
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
      var,
      manip,
      source_frame,
      target_frame,
      source_frame_offset,
      target_frame_offset,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::ABSOLUTE);
  return true;
}

bool addJointPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                const JointWaypointPoly& joint_waypoint,
                                const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                const Eigen::VectorXd& coeffs)
{
  auto constraint = createJointPositionConstraint(joint_waypoint, var, coeffs);
  nlp.addConstraintSet(constraint);
  return true;
}

bool addJointPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                 const JointWaypointPoly& joint_waypoint,
                                 const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                 const Eigen::VectorXd& coeffs)
{
  auto constraint = createJointPositionConstraint(joint_waypoint, var, coeffs);
  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}
bool addJointPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                  const JointWaypointPoly& joint_waypoint,
                                  const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                  const Eigen::VectorXd& coeffs)
{
  auto constraint = createJointPositionConstraint(joint_waypoint, var, coeffs);
  nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::ABSOLUTE);
  return true;
}

bool addCollisionConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                            const std::shared_ptr<const tesseract_environment::Environment>& env,
                            const tesseract_common::ManipulatorInfo& manip_info,
                            const std::shared_ptr<const trajopt_common::TrajOptCollisionConfig>& config,
                            const std::vector<int>& fixed_indices)
{
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices, false);
  for (auto& constraint : constraints)
    nlp.addConstraintSet(constraint);
  return true;
}

bool addCollisionCost(trajopt_sqp::QPProblem& nlp,
                      const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                      const std::shared_ptr<const tesseract_environment::Environment>& env,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const std::shared_ptr<const trajopt_common::TrajOptCollisionConfig>& config,
                      const std::vector<int>& fixed_indices)
{
  // Coefficients are applied within the constraint
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices, false);
  for (auto& constraint : constraints)
    nlp.addCostSet(constraint, trajopt_sqp::CostPenaltyType::HINGE);

  return true;
}

bool addJointVelocityConstraint(trajopt_sqp::QPProblem& nlp,
                                const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
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
                                 const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);
  nlp.addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

bool addJointAccelerationConstraint(trajopt_sqp::QPProblem& nlp,
                                    const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd accel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto accel_constraint = createJointAccelerationConstraint(accel_target, vars, coeff);
  nlp.addConstraintSet(accel_constraint);
  return true;
}

bool addJointAccelerationSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd accel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto accel_constraint = createJointAccelerationConstraint(accel_target, vars, coeff);
  nlp.addCostSet(accel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

bool addJointJerkConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                            const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd jerk_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto jerk_constraint = createJointJerkConstraint(jerk_target, vars, coeff);
  nlp.addConstraintSet(jerk_constraint);
  return true;
}

bool addJointJerkSquaredCost(trajopt_sqp::QPProblem& nlp,
                             const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                             const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd jerk_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto jerk_constraint = createJointJerkConstraint(jerk_target, vars, coeff);
  nlp.addCostSet(jerk_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  return true;
}

}  // namespace tesseract_planning

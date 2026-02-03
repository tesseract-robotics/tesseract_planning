/**
 * @file trajopt_ifopt_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_sqp/qp_problem.h>
#include <OsqpEigen/Settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

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
  // lhs.setAdaptiveRhoFraction(settings.adaptive_rho_fraction);
  lhs.setMaxIteration(static_cast<int>(settings.max_iter));
  lhs.setAbsoluteTolerance(settings.eps_abs);
  lhs.setRelativeTolerance(settings.eps_rel);
  lhs.setPrimalInfeasibilityTolerance(settings.eps_prim_inf);
  lhs.setDualInfeasibilityTolerance(settings.eps_dual_inf);
  lhs.setAlpha(settings.alpha);
  lhs.setLinearSystemSolver(settings.linsys_solver);
  lhs.setDelta(settings.delta);
  lhs.setPolish(static_cast<bool>(settings.polishing));
  lhs.setPolishRefineIter(static_cast<int>(settings.polish_refine_iter));
  lhs.setVerbosity(static_cast<bool>(settings.verbose));
  lhs.setScaledTerimination(static_cast<bool>(settings.scaled_termination));
  lhs.setCheckTermination(static_cast<int>(settings.check_termination));
  lhs.setWarmStart(static_cast<bool>(settings.warm_starting));
  // lhs.setTimeLimit(settings.time_limit);
  lhs.setAllocateSolution(static_cast<bool>(settings.allocate_solution));
  lhs.setCgMaxIter(static_cast<int>(settings.cg_max_iter));
  lhs.setCgPrecond(settings.cg_precond);
  lhs.setCgTolFraction(settings.cg_tol_fraction);
  lhs.setCgTolReduction(static_cast<int>(settings.cg_tol_reduction));
  lhs.setCheckDualGap(static_cast<bool>(settings.check_dualgap));
  lhs.setDevice(static_cast<int>(settings.device));
  lhs.setProfilerLevel(static_cast<int>(settings.profiler_level));
  lhs.setRhoIsVec(static_cast<bool>(settings.rho_is_vec));
}

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointPositionConstraint(const JointWaypointPoly& joint_waypoint,
                              const std::shared_ptr<const trajopt_ifopt::Var>& var,
                              const Eigen::VectorXd& coeffs)
{
  assert(var);

  trajopt_ifopt::ConstraintSet::Ptr constraint;
  if (!joint_waypoint.isToleranced())
  {
    constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(
        joint_waypoint.getPosition(), var, coeffs, "JointPos_" + var->getIdentifier());
  }
  else
  {
    Eigen::VectorXd lower_limit = joint_waypoint.getPosition() + joint_waypoint.getLowerTolerance();
    Eigen::VectorXd upper_limit = joint_waypoint.getPosition() + joint_waypoint.getUpperTolerance();
    auto bounds = trajopt_ifopt::toBounds(lower_limit, upper_limit);
    constraint =
        std::make_shared<trajopt_ifopt::JointPosConstraint>(bounds, var, coeffs, "JointPos_" + var->getIdentifier());
  }

  return constraint;
}

std::vector<std::shared_ptr<trajopt_ifopt::ConstraintSet>>
createCollisionConstraints(const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                           const std::shared_ptr<const tesseract_environment::Environment>& env,
                           const tesseract_common::ManipulatorInfo& manip_info,
                           const trajopt_common::TrajOptCollisionConfig& config,
                           const std::vector<int>& fixed_indices,
                           bool /*fixed_sparsity*/)
{
  std::vector<trajopt_ifopt::ConstraintSet::Ptr> constraints;
  if (config.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::NONE)
    return constraints;

  std::shared_ptr<const tesseract_kinematics::JointGroup> manip = env->getJointGroup(manip_info.manipulator);

  // Add a collision cost for all steps
  if (config.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::DISCRETE)
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      auto collision_evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(manip, env, config);

      constraints.push_back(std::make_shared<trajopt_ifopt::DiscreteCollisionConstraintD>(
          collision_evaluator, vars[i], "DiscreteCollision_" + std::to_string(i)));
    }
  }
  else if (config.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    bool time0_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), 0) != fixed_indices.end());
    for (std::size_t i = 1; i < vars.size(); ++i)
    {
      bool time1_fixed = (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end());

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSDiscreteCollisionEvaluator>(manip, env, config);

      std::array<std::shared_ptr<const trajopt_ifopt::Var>, 2> position_vars{ vars[i - 1], vars[i] };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintD>(
          collision_evaluator, position_vars, time0_fixed, time1_fixed, "LVSDiscreteCollision_" + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }
  else
  {
    const std::string prefix =
        (config.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS) ? "LVSCont"
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

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(manip, env, config);

      std::array<std::shared_ptr<const trajopt_ifopt::Var>, 2> position_vars{ vars[i - 1], vars[i] };
      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintD>(
          collision_evaluator, position_vars, time0_fixed, time1_fixed, prefix + std::to_string(i)));

      time0_fixed = time1_fixed;
    }
  }

  return constraints;
}

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                              const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                              const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(target, vars, coeffs, "JointVelocity");
  return vel_constraint;
}

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointAccelerationConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                  const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                                  const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto accel_constraint =
      std::make_shared<trajopt_ifopt::JointAccelConstraint>(target, vars, coeffs, "JointAcceleration");
  return accel_constraint;
}

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointJerkConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                          const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                          const Eigen::VectorXd& coeffs)

{
  assert(!vars.empty());
  auto jerk_constraint = std::make_shared<trajopt_ifopt::JointJerkConstraint>(target, vars, coeffs, "JointJerk");
  return jerk_constraint;
}

}  // namespace tesseract_planning

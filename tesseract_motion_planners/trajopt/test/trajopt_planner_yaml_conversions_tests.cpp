/**
 * @file trajopt_planner_yaml_conversions_tests.cpp
 * @brief This contains unit test for the tesseract trajopt planner yaml constructors
 *
 * @author Samantha Smith
 * @date August 25, 2025
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/profile_plugin_factory.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>

using namespace tesseract_planning;
using namespace trajopt_common;

TEST(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptCartesianWaypointConfig)  // NOLINT
{
  const std::string yaml_string = R"(config:
                                      enabled: false
                                      use_tolerance_override: true
                                      lower_tolerance: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                      upper_tolerance: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                      coeff: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                )";

  Eigen::VectorXd check(6);
  check << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  {  // decode

    YAML::Node n = YAML::Load(yaml_string);
    auto config = n["config"].as<TrajOptCartesianWaypointConfig>();
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.lower_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.upper_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.coeff, check));
  }

  {  // encode
    TrajOptCartesianWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n(config);

    auto output_config = output_n.as<TrajOptCartesianWaypointConfig>();
    EXPECT_EQ(output_config.enabled, config.enabled);
    EXPECT_EQ(output_config.use_tolerance_override, config.use_tolerance_override);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.lower_tolerance, config.lower_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.upper_tolerance, config.upper_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.coeff, config.coeff));
  }
}

TEST(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptJointWaypointConfig)  // NOLINT
{
  const std::string yaml_string = R"(config:
                                      enabled: false
                                      use_tolerance_override: true
                                      lower_tolerance: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                      upper_tolerance: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                      coeff: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                )";

  Eigen::VectorXd check(6);
  check << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  {  // decode
    YAML::Node n = YAML::Load(yaml_string);
    auto config = n["config"].as<TrajOptJointWaypointConfig>();
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.lower_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.upper_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.coeff, check));
  }

  {  // encode
    TrajOptJointWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n(config);

    auto output_config = output_n.as<TrajOptJointWaypointConfig>();
    EXPECT_EQ(output_config.enabled, config.enabled);
    EXPECT_EQ(output_config.use_tolerance_override, config.use_tolerance_override);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.lower_tolerance, config.lower_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.upper_tolerance, config.upper_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.coeff, config.coeff));
  }
}

TEST(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptDefaultCompositeProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultCompositeProfile profile(n["config"], plugin_factory);
    TrajOptDefaultCompositeProfile def_constructor;
    EXPECT_EQ(profile.collision_cost_config.enabled, def_constructor.collision_cost_config.enabled);
    EXPECT_EQ(profile.collision_constraint_config.enabled, def_constructor.collision_cost_config.enabled);
    EXPECT_EQ(profile.smooth_velocities, def_constructor.smooth_velocities);
    EXPECT_EQ(profile.smooth_accelerations, def_constructor.smooth_accelerations);
    EXPECT_EQ(profile.smooth_jerks, def_constructor.smooth_jerks);
    EXPECT_EQ(profile.velocity_coeff.rows(), 0);
    EXPECT_EQ(profile.acceleration_coeff.rows(), 0);
    EXPECT_EQ(profile.jerk_coeff.rows(), 0);
    EXPECT_EQ(profile.avoid_singularity, def_constructor.avoid_singularity);
    EXPECT_NEAR(profile.avoid_singularity_coeff, def_constructor.avoid_singularity_coeff, 1e-6);
  }

  {  // Constructor
    Eigen::VectorXd check(6);
    check << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    const std::string yaml_string = R"(config:
                                        collision_cost_config:
                                          enabled: false
                                        collision_constraint_config:
                                          enabled: false
                                        smooth_velocities: false
                                        smooth_accelerations: false
                                        smooth_jerks: false
                                        velocity_coeff: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                        acceleration_coeff: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                        jerk_coeff: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
                                        avoid_singularity: true
                                        avoid_singularity_coeff: 6.0
                                    )";

    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultCompositeProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.collision_cost_config.enabled, false);
    EXPECT_EQ(profile.collision_constraint_config.enabled, false);
    EXPECT_EQ(profile.smooth_velocities, false);
    EXPECT_EQ(profile.smooth_accelerations, false);
    EXPECT_EQ(profile.smooth_jerks, false);
    EXPECT_EQ(profile.avoid_singularity, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.velocity_coeff, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.acceleration_coeff, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.jerk_coeff, check));
    EXPECT_NEAR(profile.avoid_singularity_coeff, 6, 1e-8);
  }
}

TEST(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptDefaultMoveProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptDefaultMoveProfile def_constructor;
    EXPECT_EQ(profile.cartesian_cost_config.enabled, def_constructor.cartesian_cost_config.enabled);
    EXPECT_EQ(profile.cartesian_constraint_config.enabled, def_constructor.cartesian_constraint_config.enabled);
    EXPECT_EQ(profile.joint_cost_config.enabled, def_constructor.joint_cost_config.enabled);
    EXPECT_EQ(profile.joint_constraint_config.enabled, def_constructor.joint_constraint_config.enabled);
  }

  {  // Constructor
    const std::string yaml_string = R"(config:
                                        cartesian_cost_config:
                                          enabled: true
                                        cartesian_constraint_config:
                                          enabled: false
                                        joint_cost_config:
                                          enabled: true
                                        joint_constraint_config:
                                          enabled: false
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptDefaultMoveProfile def_constructor;
    EXPECT_EQ(profile.cartesian_cost_config.enabled, !def_constructor.cartesian_cost_config.enabled);
    EXPECT_EQ(profile.cartesian_constraint_config.enabled, !def_constructor.cartesian_constraint_config.enabled);
    EXPECT_EQ(profile.joint_cost_config.enabled, !def_constructor.joint_cost_config.enabled);
    EXPECT_EQ(profile.joint_constraint_config.enabled, !def_constructor.joint_constraint_config.enabled);
  }
}

bool compare(const OSQPSettings& lhs, const OSQPSettings& rhs)  // NOLINT
{
  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.rho, rhs.rho))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.sigma, rhs.sigma))
    return false;

  if (lhs.scaling != rhs.scaling)
    return false;

  if (lhs.adaptive_rho != rhs.adaptive_rho)
    return false;

  if (lhs.adaptive_rho_interval != rhs.adaptive_rho_interval)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.adaptive_rho_tolerance, rhs.adaptive_rho_tolerance))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.adaptive_rho_fraction, rhs.adaptive_rho_fraction))
    return false;

  if (lhs.max_iter != rhs.max_iter)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.eps_abs, rhs.eps_abs))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.eps_rel, rhs.eps_rel))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.eps_prim_inf, rhs.eps_prim_inf))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.eps_dual_inf, rhs.eps_dual_inf))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.alpha, rhs.alpha))
    return false;

  if (lhs.linsys_solver != rhs.linsys_solver)
    return false;

  if (lhs.delta != rhs.delta)
    return false;

  if (lhs.polishing != rhs.polishing)
    return false;

  if (lhs.polish_refine_iter != rhs.polish_refine_iter)
    return false;

  if (lhs.verbose != rhs.verbose)
    return false;

  if (lhs.scaled_termination != rhs.scaled_termination)
    return false;

  if (lhs.check_termination != rhs.check_termination)
    return false;

  if (lhs.warm_starting != rhs.warm_starting)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.time_limit, rhs.time_limit))
    return false;

  if (lhs.allocate_solution != rhs.allocate_solution)
    return false;

  if (lhs.cg_max_iter != rhs.cg_max_iter)
    return false;

  if (lhs.cg_precond != rhs.cg_precond)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.cg_tol_fraction, rhs.cg_tol_fraction))
    return false;

  if (lhs.cg_tol_reduction != rhs.cg_tol_reduction)
    return false;

  if (lhs.check_dualgap != rhs.check_dualgap)
    return false;

  if (lhs.device != rhs.device)
    return false;

  if (lhs.profiler_level != rhs.profiler_level)
    return false;

  if (lhs.rho_is_vec != rhs.rho_is_vec)
    return false;

  return true;
}

bool compare(const sco::BasicTrustRegionSQPParameters& lhs, const sco::BasicTrustRegionSQPParameters& rhs)  // NOLINT
{
  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.improve_ratio_threshold, rhs.improve_ratio_threshold))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.min_trust_box_size, rhs.min_trust_box_size))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.min_approx_improve, rhs.min_approx_improve))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.min_approx_improve_frac, rhs.min_approx_improve_frac))
    return false;

  if (lhs.max_iter != rhs.max_iter)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.trust_shrink_ratio, rhs.trust_shrink_ratio))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.trust_expand_ratio, rhs.trust_expand_ratio))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.cnt_tolerance, rhs.cnt_tolerance))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.max_merit_coeff_increases, rhs.max_merit_coeff_increases))
    return false;

  if (lhs.max_qp_solver_failures != rhs.max_qp_solver_failures)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.merit_coeff_increase_ratio, rhs.merit_coeff_increase_ratio))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.max_time, rhs.max_time))
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.initial_merit_error_coeff, rhs.initial_merit_error_coeff))
    return false;

  if (lhs.inflate_constraints_individually != rhs.inflate_constraints_individually)
    return false;

  if (!tesseract_common::almostEqualRelativeAndAbs(lhs.trust_box_size, rhs.trust_box_size))
    return false;

  if (lhs.log_results != rhs.log_results)
    return false;

  if (lhs.log_dir != rhs.log_dir)
    return false;

  if (lhs.num_threads != rhs.num_threads)
    return false;

  return true;
}

TEST(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptOSQPSolverProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptOSQPSolverProfile def_constructor;
    EXPECT_EQ(profile.update_workspace, def_constructor.update_workspace);
    EXPECT_TRUE(compare(profile.settings, def_constructor.settings));
    EXPECT_TRUE(compare(profile.opt_params, def_constructor.opt_params));
  }

  {  // Constructor
    const std::string yaml_string = R"(config:
                                        update_workspace: false
                                        settings:
                                          rho: 0.5
                                          sigma: 1
                                          scaling: 15
                                          adaptive_rho: 2
                                          adaptive_rho_interval: 3
                                          adaptive_rho_tolerance: 4
                                          adaptive_rho_fraction: 0.2
                                          max_iter: 20
                                          eps_abs: 6
                                          eps_rel: 9
                                          eps_prim_inf: 12
                                          eps_dual_inf: 14
                                          alpha: 3.6
                                          linsys_solver: 'OSQP_INDIRECT_SOLVER'
                                          delta: 0.1
                                          polishing: 1
                                          polish_refine_iter: 17
                                          verbose: 0
                                          scaled_termination: 40
                                          check_termination: 28
                                          warm_starting: 0
                                          time_limit: 60
                                          allocate_solution: 0
                                          cg_max_iter: 30
                                          cg_precond: 'OSQP_NO_PRECONDITIONER'
                                          cg_tol_fraction: 0.1
                                          cg_tol_reduction: 5
                                          check_dualgap: 1
                                          device: 1
                                          profiler_level: 1
                                          rho_is_vec: 0
                                        opt_params:
                                          improve_ratio_threshold: 0.1
                                          min_trust_box_size: 1
                                          min_approx_improve: 2
                                          min_approx_improve_frac: 0
                                          max_iter: 20
                                          trust_shrink_ratio: 0.5
                                          trust_expand_ratio: 2.5
                                          cnt_tolerance: 3
                                          max_merit_coeff_increases: 10
                                          max_qp_solver_failures: 5
                                          merit_coeff_increase_ratio: 6
                                          max_time: 1
                                          initial_merit_error_coeff: 4
                                          inflate_constraints_individually: false
                                          trust_box_size: 7
                                          log_results: true
                                          log_dir: '/temp'
                                          num_threads: 100
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptOSQPSolverProfile def_constructor;

    def_constructor.settings.rho = 0.5;
    def_constructor.settings.sigma = 1;
    def_constructor.settings.scaling = 15;
    def_constructor.settings.adaptive_rho = 2;
    def_constructor.settings.adaptive_rho_interval = 3;
    def_constructor.settings.adaptive_rho_tolerance = 4;
    def_constructor.settings.adaptive_rho_fraction = 0.2;
    def_constructor.settings.max_iter = 20;
    def_constructor.settings.eps_abs = 6;
    def_constructor.settings.eps_rel = 9;
    def_constructor.settings.eps_prim_inf = 12;
    def_constructor.settings.eps_dual_inf = 14;
    def_constructor.settings.alpha = 3.6;
    def_constructor.settings.linsys_solver = OSQP_INDIRECT_SOLVER;
    def_constructor.settings.delta = 0.1;
    def_constructor.settings.polishing = 1;
    def_constructor.settings.polish_refine_iter = 17;
    def_constructor.settings.verbose = 0;
    def_constructor.settings.scaled_termination = 40;
    def_constructor.settings.check_termination = 28;
    def_constructor.settings.warm_starting = 0;
    def_constructor.settings.time_limit = 60;
    // OSQP v1.0.0 params
    def_constructor.settings.allocate_solution = 0;
    def_constructor.settings.cg_max_iter = 30;
    def_constructor.settings.cg_precond = OSQP_NO_PRECONDITIONER;
    def_constructor.settings.cg_tol_fraction = 0.1;
    def_constructor.settings.cg_tol_reduction = 5;
    def_constructor.settings.check_dualgap = 1;
    def_constructor.settings.device = 1;
    def_constructor.settings.profiler_level = 1;
    def_constructor.settings.rho_is_vec = 0;

    def_constructor.opt_params.improve_ratio_threshold = 0.1;
    def_constructor.opt_params.min_trust_box_size = 1;
    def_constructor.opt_params.min_approx_improve = 2;
    def_constructor.opt_params.min_approx_improve_frac = 0;
    def_constructor.opt_params.max_iter = 20;
    def_constructor.opt_params.trust_shrink_ratio = 0.5;
    def_constructor.opt_params.trust_expand_ratio = 2.5;
    def_constructor.opt_params.cnt_tolerance = 3;
    def_constructor.opt_params.max_merit_coeff_increases = 10;
    def_constructor.opt_params.max_qp_solver_failures = 5;
    def_constructor.opt_params.merit_coeff_increase_ratio = 6;
    def_constructor.opt_params.max_time = 1;
    def_constructor.opt_params.initial_merit_error_coeff = 4;
    def_constructor.opt_params.inflate_constraints_individually = false;
    def_constructor.opt_params.trust_box_size = 7;
    def_constructor.opt_params.log_results = true;
    def_constructor.opt_params.log_dir = "/temp";
    def_constructor.opt_params.num_threads = 100;

    EXPECT_EQ(profile.update_workspace, false);
    EXPECT_TRUE(compare(profile.settings, def_constructor.settings));
    EXPECT_TRUE(compare(profile.opt_params, def_constructor.opt_params));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

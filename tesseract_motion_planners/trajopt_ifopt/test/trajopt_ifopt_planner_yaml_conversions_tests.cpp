/**
 * @file trajopt_planner_yaml_conversions_tests.cpp
 * @brief This contains unit test for the tesseract trajopt planner yaml constructors
 *
 * @author Samantha Smith
 * @date August 25, 2025
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
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>
#include <tesseract_motion_planners/trajopt_ifopt/cereal_serialization.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>

using namespace tesseract_planning;
using namespace trajopt_common;

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, Serialization)  // NOLINT
{
  // Create Profiles
  auto move_profile = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptIfoptOSQPSolverProfile>();

  // Serialization
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptIfoptDefaultMoveProfile>(
      move_profile, "trajopt_ifopt_move_profile");
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptIfoptDefaultCompositeProfile>(
      composite_profile, "trajopt_ifopt_composite_profile");
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptIfoptOSQPSolverProfile>(
      solver_profile, "trajopt_ifopt_solver_profile");
}

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, TrajOptIfoptCartesianWaypointConfig)  // NOLINT
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
    auto config = n["config"].as<TrajOptIfoptCartesianWaypointConfig>();
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.lower_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.upper_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.coeff, check));
  }

  {  // encode
    TrajOptIfoptCartesianWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n(config);

    auto output_config = output_n.as<TrajOptIfoptCartesianWaypointConfig>();
    EXPECT_EQ(output_config.enabled, config.enabled);
    EXPECT_EQ(output_config.use_tolerance_override, config.use_tolerance_override);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.lower_tolerance, config.lower_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.upper_tolerance, config.upper_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.coeff, config.coeff));
  }
}

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, TrajOptIfoptJointWaypointConfig)  // NOLINT
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
    auto config = n["config"].as<TrajOptIfoptJointWaypointConfig>();
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.lower_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.upper_tolerance, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.coeff, check));
  }

  {  // encode
    TrajOptIfoptJointWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n(config);

    auto output_config = output_n.as<TrajOptIfoptJointWaypointConfig>();
    EXPECT_EQ(output_config.enabled, config.enabled);
    EXPECT_EQ(output_config.use_tolerance_override, config.use_tolerance_override);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.lower_tolerance, config.lower_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.upper_tolerance, config.upper_tolerance));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(output_config.coeff, config.coeff));
  }
}

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, TrajOptIfoptDefaultCompositeProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptIfoptDefaultCompositeProfile profile(n["config"], plugin_factory);
    TrajOptIfoptDefaultCompositeProfile def_constructor;
    EXPECT_EQ(profile.collision_cost_config.enabled, def_constructor.collision_cost_config.enabled);
    EXPECT_EQ(profile.collision_constraint_config.enabled, def_constructor.collision_cost_config.enabled);
    EXPECT_EQ(profile.smooth_velocities, def_constructor.smooth_velocities);
    EXPECT_EQ(profile.smooth_accelerations, def_constructor.smooth_accelerations);
    EXPECT_EQ(profile.smooth_jerks, def_constructor.smooth_jerks);
    EXPECT_EQ(profile.velocity_coeff.rows(), 0);
    EXPECT_EQ(profile.acceleration_coeff.rows(), 0);
    EXPECT_EQ(profile.jerk_coeff.rows(), 0);
    // EXPECT_EQ(profile.avoid_singularity, def_constructor.avoid_singularity);
    // EXPECT_NEAR(profile.avoid_singularity_coeff, def_constructor.avoid_singularity_coeff, 1e-6);
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
    TrajOptIfoptDefaultCompositeProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.collision_cost_config.enabled, false);
    EXPECT_EQ(profile.collision_constraint_config.enabled, false);
    EXPECT_EQ(profile.smooth_velocities, false);
    EXPECT_EQ(profile.smooth_accelerations, false);
    EXPECT_EQ(profile.smooth_jerks, false);
    // EXPECT_EQ(profile.avoid_singularity, true);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.velocity_coeff, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.acceleration_coeff, check));
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(profile.jerk_coeff, check));
    // EXPECT_NEAR(profile.avoid_singularity_coeff, 6, 1e-8);
  }
}

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, TrajOptIfoptDefaultMoveProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptIfoptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptIfoptDefaultMoveProfile def_constructor;
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
    TrajOptIfoptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptIfoptDefaultMoveProfile def_constructor;
    EXPECT_EQ(profile.cartesian_cost_config.enabled, !def_constructor.cartesian_cost_config.enabled);
    EXPECT_EQ(profile.cartesian_constraint_config.enabled, !def_constructor.cartesian_constraint_config.enabled);
    EXPECT_EQ(profile.joint_cost_config.enabled, !def_constructor.joint_cost_config.enabled);
    EXPECT_EQ(profile.joint_constraint_config.enabled, !def_constructor.joint_constraint_config.enabled);
  }
}

TEST(TesseractPlanningTrajoptIfoptYAMLConversionsUnit, TrajOptIfoptOSQPSolverProfile)  // NOLINT
{
  {  // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptIfoptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptIfoptOSQPSolverProfile def_constructor;
    // EXPECT_EQ(profile.update_workspace, def_constructor.update_workspace);
    EXPECT_TRUE(*profile.qp_settings == *def_constructor.qp_settings);
    EXPECT_TRUE(profile.opt_params == def_constructor.opt_params);
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
                                          max_iterations: 20
                                          trust_shrink_ratio: 0.5
                                          trust_expand_ratio: 2.5
                                          cnt_tolerance: 3
                                          max_merit_coeff_increases: 10
                                          max_qp_solver_failures: 5
                                          merit_coeff_increase_ratio: 6
                                          max_time: 1
                                          initial_merit_error_coeff: 4
                                          inflate_constraints_individually: false
                                          initial_trust_box_size: 7
                                          log_results: true
                                          log_dir: '/temp'
                                          num_threads: 100
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptIfoptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptIfoptOSQPSolverProfile def_constructor;

    auto& settings = *def_constructor.qp_settings->getSettings();
    settings.rho = 0.5;
    settings.sigma = 1;
    settings.scaling = 15;
    settings.adaptive_rho = 2;
    settings.adaptive_rho_interval = 3;
    settings.adaptive_rho_tolerance = 4;
    settings.adaptive_rho_fraction = 0.2;
    settings.max_iter = 20;
    settings.eps_abs = 6;
    settings.eps_rel = 9;
    settings.eps_prim_inf = 12;
    settings.eps_dual_inf = 14;
    settings.alpha = 3.6;
    settings.linsys_solver = OSQP_INDIRECT_SOLVER;
    settings.delta = 0.1;
    settings.polishing = 1;
    settings.polish_refine_iter = 17;
    settings.verbose = 0;
    settings.scaled_termination = 40;
    settings.check_termination = 28;
    settings.warm_starting = 0;
    settings.time_limit = 60;
    settings.check_dualgap = 0;
    // OSQP v1.0.0 params
    settings.allocate_solution = 0;
    settings.cg_max_iter = 30;
    settings.cg_precond = OSQP_NO_PRECONDITIONER;
    settings.cg_tol_fraction = 0.1;
    settings.cg_tol_reduction = 5;
    settings.check_dualgap = 1;
    settings.device = 1;
    settings.profiler_level = 1;
    settings.rho_is_vec = 0;

    def_constructor.opt_params.improve_ratio_threshold = 0.1;
    def_constructor.opt_params.min_trust_box_size = 1;
    def_constructor.opt_params.min_approx_improve = 2;
    def_constructor.opt_params.min_approx_improve_frac = 0;
    def_constructor.opt_params.max_iterations = 20;
    def_constructor.opt_params.trust_shrink_ratio = 0.5;
    def_constructor.opt_params.trust_expand_ratio = 2.5;
    def_constructor.opt_params.cnt_tolerance = 3;
    def_constructor.opt_params.max_merit_coeff_increases = 10;
    def_constructor.opt_params.max_qp_solver_failures = 5;
    def_constructor.opt_params.merit_coeff_increase_ratio = 6;
    def_constructor.opt_params.max_time = 1;
    def_constructor.opt_params.initial_merit_error_coeff = 4;
    def_constructor.opt_params.inflate_constraints_individually = false;
    def_constructor.opt_params.initial_trust_box_size = 7;
    def_constructor.opt_params.log_results = true;
    def_constructor.opt_params.log_dir = "/temp";
    // def_constructor.opt_params.num_threads = 100;

    // EXPECT_EQ(profile.update_workspace, def_constructor.update_workspace);
    EXPECT_TRUE(*profile.qp_settings == *def_constructor.qp_settings);
    EXPECT_TRUE(profile.opt_params == def_constructor.opt_params);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

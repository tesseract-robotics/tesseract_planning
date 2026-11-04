/**
 * @file trajopt_ifopt_utils_unit.cpp
 * @brief Unit tests for createCollisionConstraints
 *
 * @author Jelle Feringa
 * @date November 2, 2026
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <filesystem>
#include <numeric>
#include <trajopt_common/collision_types.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

using namespace tesseract::motion_planners;
using tesseract::collision::CollisionEvaluatorType;

/** @brief Program length: a fixed first waypoint and several free segments, so a shared evaluator is observable */
static constexpr std::size_t N_WAYPOINTS = 5;

class TesseractPlanningTrajoptIfoptUtilsUnit : public ::testing::TestWithParam<CollisionEvaluatorType>
{
protected:
  std::shared_ptr<tesseract::environment::Environment> env_;
  tesseract::common::ManipulatorInfo manip_info_{ "manipulator", "base_link", "tool0" };
  std::shared_ptr<trajopt_ifopt::NodesVariables> nodes_variables_;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    env_ = std::make_shared<tesseract::environment::Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    ASSERT_TRUE(env_->init(urdf_path, srdf_path, locator));

    auto manip = env_->getJointGroup(manip_info_.manipulator);
    const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);
    const Eigen::VectorXd pos = Eigen::VectorXd::Zero(manip->numJoints());

    std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
    for (std::size_t i = 0; i < N_WAYPOINTS; ++i)
    {
      nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_" + std::to_string(i)));
      vars_.push_back(nodes.back()->addVar("position", tesseract::common::toNames(manip->getJointIds()), pos, bounds));
    }
    nodes_variables_ = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  }

  static trajopt_common::TrajOptCollisionConfig makeConfig(CollisionEvaluatorType type, double margin)
  {
    trajopt_common::TrajOptCollisionConfig config(margin, 1);
    config.collision_check_config.type = type;
    return config;
  }
};

/** @brief The evaluator a collision constraint holds, as a type-erased pointer */
static const void* evaluatorOf(const std::shared_ptr<trajopt_ifopt::ConstraintSet>& constraint)
{
  if (auto discrete = std::dynamic_pointer_cast<trajopt_ifopt::DiscreteCollisionConstraintD>(constraint))
    return discrete->getCollisionEvaluator().get();
  if (auto continuous = std::dynamic_pointer_cast<trajopt_ifopt::ContinuousCollisionConstraintD>(constraint))
    return continuous->getCollisionEvaluator().get();
  return nullptr;
}

static std::string prefixOf(CollisionEvaluatorType type)
{
  switch (type)
  {
    case CollisionEvaluatorType::DISCRETE:
      return "DiscreteCollision_";
    case CollisionEvaluatorType::LVS_DISCRETE:
      return "LVSDiscreteCollision_";
    case CollisionEvaluatorType::LVS_CONTINUOUS:
      return "LVSContinuousCollision_";
    default:
      return "ContinuousCollision_";
  }
}

/** @brief Every constraint of one call holds the same evaluator, so the contact manager is cloned once per call */
TEST_P(TesseractPlanningTrajoptIfoptUtilsUnit, OneEvaluatorPerCall)  // NOLINT
{
  const CollisionEvaluatorType type = GetParam();
  const std::vector<int> fixed_indices{ 0 };
  auto constraints =
      createCollisionConstraints(vars_, env_, manip_info_, makeConfig(type, 0.025), fixed_indices, false);

  // DISCRETE skips the fixed waypoint 0; the segment types add one term per segment 1..N-1.
  ASSERT_EQ(constraints.size(), N_WAYPOINTS - 1);
  for (std::size_t k = 0; k < constraints.size(); ++k)
    EXPECT_EQ(constraints[k]->getName(), prefixOf(type) + std::to_string(k + 1));

  const void* shared = evaluatorOf(constraints.front());
  ASSERT_NE(shared, nullptr);
  for (const auto& constraint : constraints)
    EXPECT_EQ(evaluatorOf(constraint), shared) << constraint->getName();
}

/** @brief Separate calls (the cost and the constraint config) never share an evaluator */
TEST_P(TesseractPlanningTrajoptIfoptUtilsUnit, SeparateEvaluatorPerConfig)  // NOLINT
{
  const CollisionEvaluatorType type = GetParam();
  auto cost = createCollisionConstraints(vars_, env_, manip_info_, makeConfig(type, 0.025), {}, false);
  auto constraint = createCollisionConstraints(vars_, env_, manip_info_, makeConfig(type, 0.01), {}, false);
  ASSERT_FALSE(cost.empty());
  ASSERT_FALSE(constraint.empty());
  EXPECT_NE(evaluatorOf(cost.front()), evaluatorOf(constraint.front()));
}

/** @brief One waypoint: DISCRETE adds its term, the segment types add none (there is no segment) */
TEST_P(TesseractPlanningTrajoptIfoptUtilsUnit, OneWaypoint)  // NOLINT
{
  const CollisionEvaluatorType type = GetParam();
  const std::vector<std::shared_ptr<const trajopt_ifopt::Var>> one_var{ vars_.front() };
  auto constraints = createCollisionConstraints(one_var, env_, manip_info_, makeConfig(type, 0.025), {}, false);

  const std::size_t expected = (type == CollisionEvaluatorType::DISCRETE) ? 1 : 0;
  ASSERT_EQ(constraints.size(), expected);
  if (expected == 1)
  {
    EXPECT_EQ(constraints.front()->getName(), prefixOf(type) + "0");
    EXPECT_NE(evaluatorOf(constraints.front()), nullptr);
  }
}

INSTANTIATE_TEST_SUITE_P(CollisionEvaluatorTypes,
                         TesseractPlanningTrajoptIfoptUtilsUnit,
                         ::testing::Values(CollisionEvaluatorType::DISCRETE,
                                           CollisionEvaluatorType::LVS_DISCRETE,
                                           CollisionEvaluatorType::LVS_CONTINUOUS,
                                           CollisionEvaluatorType::CONTINUOUS));

/** @brief A fully fixed DISCRETE program and type NONE add no collision terms */
TEST_F(TesseractPlanningTrajoptIfoptUtilsUnit, NoTermsWhenFixedOrNone)  // NOLINT
{
  std::vector<int> all_fixed(N_WAYPOINTS);
  std::iota(all_fixed.begin(), all_fixed.end(), 0);
  EXPECT_TRUE(createCollisionConstraints(
                  vars_, env_, manip_info_, makeConfig(CollisionEvaluatorType::DISCRETE, 0.025), all_fixed, false)
                  .empty());
  EXPECT_TRUE(
      createCollisionConstraints(vars_, env_, manip_info_, makeConfig(CollisionEvaluatorType::NONE, 0.025), {}, false)
          .empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

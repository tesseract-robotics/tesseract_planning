#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <trajopt_common/collision_types.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/types.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

using namespace tesseract::motion_planners;
using tesseract::collision::CollisionEvaluatorType;

class TesseractMotionPlannersTrajoptIfoptUtilsUnit : public ::testing::Test
{
protected:
  tesseract::environment::Environment::Ptr env_;
  tesseract::common::ManipulatorInfo manip_;
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes_;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    const std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    env_ = std::make_shared<tesseract::environment::Environment>();
    ASSERT_TRUE(env_->init(urdf_path, srdf_path, locator));
    manip_.manipulator = "manipulator";

    const std::vector<std::string> joint_names =
        tesseract::common::toNames(env_->getJointGroup(manip_.manipulator)->getJointIds());
    const auto n_dof = static_cast<Eigen::Index>(joint_names.size());
    const std::vector<trajopt_ifopt::Bounds> bounds(joint_names.size(), trajopt_ifopt::NoBound);
    for (int i = 0; i < 4; ++i)
    {
      nodes_.push_back(std::make_unique<trajopt_ifopt::Node>("Node_" + std::to_string(i)));
      vars_.push_back(nodes_.back()->addVar("position", joint_names, Eigen::VectorXd::Zero(n_dof), bounds));
    }
  }

  std::vector<std::string> constraintNames(CollisionEvaluatorType type, const std::vector<int>& fixed_indices) const
  {
    trajopt_common::TrajOptCollisionConfig config;
    config.collision_check_config.type = type;

    std::vector<std::string> names;
    for (const auto& cnt : createCollisionConstraints(vars_, env_, manip_, config, fixed_indices, false))
      names.push_back(cnt->getName());
    return names;
  }
};

TEST_F(TesseractMotionPlannersTrajoptIfoptUtilsUnit, CollisionConstraintsSkipFixedSegments)  // NOLINT
{
  // States 0 and 1 are fixed, so only the segments ending at states 2 and 3 have a free variable
  const std::vector<int> fixed_indices{ 0, 1, 3 };

  EXPECT_EQ(constraintNames(CollisionEvaluatorType::LVS_DISCRETE, fixed_indices),
            (std::vector<std::string>{ "LVSDiscreteCollision_2", "LVSDiscreteCollision_3" }));
  EXPECT_EQ(constraintNames(CollisionEvaluatorType::LVS_CONTINUOUS, fixed_indices),
            (std::vector<std::string>{ "LVSContinuousCollision_2", "LVSContinuousCollision_3" }));
  EXPECT_EQ(constraintNames(CollisionEvaluatorType::CONTINUOUS, fixed_indices),
            (std::vector<std::string>{ "ContinuousCollision_2", "ContinuousCollision_3" }));
  EXPECT_EQ(constraintNames(CollisionEvaluatorType::DISCRETE, fixed_indices),
            (std::vector<std::string>{ "DiscreteCollision_2" }));
}

TEST_F(TesseractMotionPlannersTrajoptIfoptUtilsUnit, CollisionConstraintsAllFixed)  // NOLINT
{
  const std::vector<int> fixed_indices{ 0, 1, 2, 3 };

  EXPECT_TRUE(constraintNames(CollisionEvaluatorType::LVS_DISCRETE, fixed_indices).empty());
  EXPECT_TRUE(constraintNames(CollisionEvaluatorType::LVS_CONTINUOUS, fixed_indices).empty());
  EXPECT_TRUE(constraintNames(CollisionEvaluatorType::CONTINUOUS, fixed_indices).empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

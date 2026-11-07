#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <filesystem>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/motion_planners/planner_utils.h>

using tesseract::common::LinkId;
using tesseract::environment::Environment;
using tesseract::kinematics::JointGroup;
using tesseract::motion_planners::isValidState;

class PlannerUtilsIsValidStateUnit : public testing::Test
{
protected:
  Environment env;
  JointGroup::ConstPtr joint_group;
  Eigen::MatrixX2d limits;

  void SetUp() override
  {
    const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const std::filesystem::path urdf(
        locator->locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")->getFilePath());
    const std::filesystem::path srdf(
        locator->locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")->getFilePath());
    ASSERT_TRUE(env.init(urdf, srdf, locator));
    joint_group = env.getJointGroup("manipulator");
    limits = joint_group->getLimits().joint_limits;
  }

  bool valid(const Eigen::VectorXd& joint_values) const
  {
    return isValidState<double>(*joint_group, LinkId("base_link"), LinkId("tool0"), joint_values, limits);
  }
};

TEST_F(PlannerUtilsIsValidStateUnit, FlipUpFrontIsValid)  // NOLINT
{
  // Home: tool0 in front of joint 1, wrist and elbow at zero
  EXPECT_TRUE(valid(Eigen::VectorXd::Zero(6)));
}

TEST_F(PlannerUtilsIsValidStateUnit, NoFlipUpFrontIsValid)  // NOLINT
{
  Eigen::VectorXd joint_values = Eigen::VectorXd::Zero(6);
  joint_values(4) = -0.5;  // joint 5 negative: wrist not flipped
  EXPECT_TRUE(valid(joint_values));
}

TEST_F(PlannerUtilsIsValidStateUnit, BackConfigurationIsInvalid)  // NOLINT
{
  Eigen::VectorXd joint_values = Eigen::VectorXd::Zero(6);
  joint_values(1) = -1.7;  // joint 2 leaned back, inside its lower limit of -1.7453: tool0 behind joint 1
  EXPECT_FALSE(valid(joint_values));
}

TEST_F(PlannerUtilsIsValidStateUnit, OutsideJointLimitsIsInvalid)  // NOLINT
{
  Eigen::VectorXd joint_values = Eigen::VectorXd::Zero(6);
  joint_values(2) = 1.2;  // joint 3 above its upper limit of 1.1345
  EXPECT_FALSE(valid(joint_values));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

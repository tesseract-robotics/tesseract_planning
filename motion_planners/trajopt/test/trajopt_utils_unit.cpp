#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/motion_planners/trajopt/trajopt_utils.h>

using tesseract::common::JointId;
using tesseract::motion_planners::createNearJointStateTermInfo;

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CreateNearJointStateTermInfoIds)  // NOLINT
{
  Eigen::VectorXd target(2);
  target << 0.1, 0.2;
  const std::vector<JointId> joint_ids{ JointId("j1"), JointId("j2") };
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 5.0);

  const auto term = createNearJointStateTermInfo(target, joint_ids, 3, coeffs, trajopt::TermType::TT_COST);

  const auto jp = std::dynamic_pointer_cast<trajopt::JointPosTermInfo>(term);
  ASSERT_NE(jp, nullptr);
  EXPECT_EQ(jp->coeffs.size(), 2);
  EXPECT_EQ(jp->targets.size(), 2);
  EXPECT_DOUBLE_EQ(jp->coeffs.at(1), 5.0);
  EXPECT_DOUBLE_EQ(jp->targets.at(1), 0.2);
  EXPECT_EQ(jp->first_step, 3);
  EXPECT_EQ(jp->last_step, 3);
  EXPECT_EQ(jp->name, "near_state_3");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

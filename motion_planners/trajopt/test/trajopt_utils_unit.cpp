#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/motion_planners/trajopt/trajopt_utils.h>

using tesseract::common::JointId;
using tesseract::common::LinkId;
using tesseract::motion_planners::createAvoidSingularityTermInfo;
using tesseract::motion_planners::createCartesianWaypointTermInfo;
using tesseract::motion_planners::createDynamicCartesianWaypointTermInfo;
using tesseract::motion_planners::createJointWaypointTermInfo;
using tesseract::motion_planners::createNearJointStateTermInfo;
using tesseract::motion_planners::createTolerancedJointWaypointTermInfo;

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CreateNearJointStateTermInfoIds)  // NOLINT
{
  Eigen::VectorXd target(2);
  target << 0.1, 0.2;
  const std::vector<JointId> joint_ids{ "j1", "j2" };
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

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CreateNearJointStateTermInfoPerJointCoeffs)  // NOLINT
{
  Eigen::VectorXd target(2);
  target << 0.1, 0.2;
  const std::vector<JointId> joint_ids{ "j1", "j2" };
  Eigen::VectorXd coeffs(2);
  coeffs << 5.0, 7.0;

  const auto term = createNearJointStateTermInfo(target, joint_ids, 3, coeffs, trajopt::TermType::TT_COST);

  const auto jp = std::dynamic_pointer_cast<trajopt::JointPosTermInfo>(term);
  ASSERT_NE(jp, nullptr);
  ASSERT_EQ(jp->coeffs.size(), 2);
  EXPECT_DOUBLE_EQ(jp->coeffs.at(0), 5.0);
  EXPECT_DOUBLE_EQ(jp->coeffs.at(1), 7.0);
}

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CreateDynamicCartesianWaypointTermInfoIds)  // NOLINT
{
  const LinkId working_frame{ "base_link" };
  const LinkId tcp_frame{ "tool0" };
  Eigen::Isometry3d c_wp{ Eigen::Isometry3d::Identity() };
  c_wp.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Isometry3d tcp_offset{ Eigen::Isometry3d::Identity() };
  tcp_offset.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 5.0);

  const auto term = createDynamicCartesianWaypointTermInfo(
      4, working_frame, c_wp, tcp_frame, tcp_offset, coeffs, trajopt::TermType::TT_CNT);

  const auto pose = std::dynamic_pointer_cast<trajopt::DynamicCartPoseTermInfo>(term);
  ASSERT_NE(pose, nullptr);
  EXPECT_EQ(pose->source_frame, tcp_frame);
  EXPECT_EQ(pose->target_frame, working_frame);
  EXPECT_TRUE(pose->source_frame_offset.isApprox(tcp_offset));
  EXPECT_TRUE(pose->target_frame_offset.isApprox(c_wp));
  EXPECT_EQ(pose->timestep, 4);
  EXPECT_EQ(pose->name, "dyn_cartesian_waypoint_4");
}

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CreateAvoidSingularityTermInfoIds)  // NOLINT
{
  const LinkId link{ "tool0" };

  const auto term = createAvoidSingularityTermInfo(0, 5, link, 3.0, trajopt::TermType::TT_COST);

  const auto as = std::dynamic_pointer_cast<trajopt::AvoidSingularityTermInfo>(term);
  ASSERT_NE(as, nullptr);
  EXPECT_EQ(as->link_id, link);
  EXPECT_EQ(as->first_step, 0);
  EXPECT_EQ(as->last_step, 5);
  ASSERT_EQ(as->coeffs.size(), 1);
  EXPECT_DOUBLE_EQ(as->coeffs.at(0), 3.0);
  EXPECT_EQ(as->name, "avoid_singularity");
}

TEST(TesseractMotionPlannersTrajoptUtilsUnit, TermInfoFactoriesRejectBadCoeffs)  // NOLINT
{
  const LinkId working_frame{ "base_link" };
  const LinkId tcp_frame{ "tool0" };
  const Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  Eigen::VectorXd bad(3);
  bad << 1.0, 2.0, 3.0;

  // Cartesian terms accept one weight shared by all six degrees of freedom or one weight each
  EXPECT_THROW(
      createCartesianWaypointTermInfo(0, working_frame, pose, tcp_frame, pose, bad, trajopt::TermType::TT_COST),
      std::runtime_error);
  EXPECT_THROW(
      createDynamicCartesianWaypointTermInfo(0, working_frame, pose, tcp_frame, pose, bad, trajopt::TermType::TT_COST),
      std::runtime_error);

  // Joint terms accept one weight shared by all joints or one weight each
  Eigen::VectorXd j_wp(2);
  j_wp << 0.1, 0.2;
  const std::vector<JointId> joint_ids{ "j1", "j2" };
  EXPECT_THROW(createJointWaypointTermInfo(j_wp, 0, bad, trajopt::TermType::TT_COST), std::runtime_error);
  EXPECT_THROW(createTolerancedJointWaypointTermInfo(j_wp, j_wp, j_wp, 0, bad, trajopt::TermType::TT_COST),
               std::runtime_error);
  EXPECT_THROW(createNearJointStateTermInfo(j_wp, joint_ids, 0, bad, trajopt::TermType::TT_COST), std::runtime_error);
}

TEST(TesseractMotionPlannersTrajoptUtilsUnit, CartesianWaypointTermInfoRejectsBadTolerances)  // NOLINT
{
  const LinkId working_frame{ "base_link" };
  const LinkId tcp_frame{ "tool0" };
  const Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 5.0);
  const Eigen::VectorXd good = Eigen::VectorXd::Constant(6, 0.01);
  Eigen::VectorXd bad(3);
  bad << 1.0, 2.0, 3.0;

  // A tolerance may be omitted, but when given it covers one or all six degrees of freedom
  EXPECT_THROW(
      createCartesianWaypointTermInfo(0, working_frame, pose, tcp_frame, pose, coeffs, trajopt::TermType::TT_COST, bad),
      std::runtime_error);
  EXPECT_THROW(createCartesianWaypointTermInfo(
                   0, working_frame, pose, tcp_frame, pose, coeffs, trajopt::TermType::TT_COST, good, bad),
               std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

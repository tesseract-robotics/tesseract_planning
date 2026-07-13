#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/poly/joint_waypoint_poly.h>
#include <tesseract/motion_planners/simple/interpolation.h>

using tesseract::command_language::JointWaypointPoly;
using tesseract::command_language::MoveInstruction;
using tesseract::command_language::MoveInstructionType;
using tesseract::command_language::StateWaypoint;
using tesseract::common::JointId;
using tesseract::motion_planners::getInterpolatedInstructions;

namespace
{
/** @brief Three columns of a two-joint trajectory: start, middle, end */
Eigen::MatrixXd states()
{
  Eigen::MatrixXd s(2, 3);
  s << 0.0, 0.5, 1.0, 0.0, 0.5, 1.0;
  return s;
}

MoveInstruction baseInstruction(const std::vector<JointId>& joint_ids)
{
  return MoveInstruction(StateWaypoint(joint_ids, Eigen::VectorXd::Zero(2)), MoveInstructionType::FREESPACE);
}
}  // namespace

TEST(TesseractMotionPlannersSimpleInterpolationUnit, GetInterpolatedInstructionsIds)  // NOLINT
{
  const std::vector<JointId> joint_ids{ JointId("j1"), JointId("j2") };

  const auto mis = getInterpolatedInstructions(joint_ids, states(), baseInstruction(joint_ids));

  ASSERT_EQ(mis.size(), 2);
  const auto& jwp = mis.front().getWaypoint().as<JointWaypointPoly>();
  EXPECT_EQ(jwp.getJointIds(), joint_ids);
  EXPECT_FALSE(jwp.isConstrained());
  EXPECT_NEAR(jwp.getPosition()(0), 0.5, 1e-12);
}

TEST(TesseractMotionPlannersSimpleInterpolationUnit, GetInterpolatedInstructionsNamesMatchIds)  // NOLINT
{
  const std::vector<std::string> joint_names{ "j1", "j2" };
  const std::vector<JointId> joint_ids = tesseract::common::toIds<JointId>(joint_names);

  const auto mis = getInterpolatedInstructions(joint_names, states(), baseInstruction(joint_ids));

  ASSERT_EQ(mis.size(), 2);
  EXPECT_EQ(mis.front().getWaypoint().as<JointWaypointPoly>().getJointIds(), joint_ids);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/motion_planners/planner.h>

using tesseract::command_language::CartesianWaypoint;
using tesseract::command_language::CartesianWaypointPoly;
using tesseract::command_language::MoveInstruction;
using tesseract::command_language::MoveInstructionPoly;
using tesseract::command_language::MoveInstructionType;
using tesseract::command_language::StateWaypointPoly;
using tesseract::common::JointId;
using tesseract::motion_planners::MotionPlanner;

namespace
{
Eigen::VectorXd values()
{
  Eigen::VectorXd v(2);
  v << 1.0, 2.0;
  return v;
}
}  // namespace

TEST(TesseractMotionPlannersCoreUnit, AssignSolutionIdsCreatesStateWaypoint)  // NOLINT
{
  const std::vector<JointId> joint_ids{ "j1", "j2" };
  MoveInstructionPoly mi(
      MoveInstruction(CartesianWaypoint(Eigen::Isometry3d::Identity()), MoveInstructionType::FREESPACE));

  MotionPlanner::assignSolution(mi, joint_ids, values(), false);

  ASSERT_TRUE(mi.getWaypoint().isStateWaypoint());
  const auto& swp = mi.getWaypoint().as<StateWaypointPoly>();
  EXPECT_EQ(swp.getJointIds(), joint_ids);
  EXPECT_TRUE(swp.getPosition().isApprox(values()));
}

TEST(TesseractMotionPlannersCoreUnit, AssignSolutionIdsSeedsCartesianWaypoint)  // NOLINT
{
  const std::vector<JointId> joint_ids{ "j1", "j2" };
  MoveInstructionPoly mi(
      MoveInstruction(CartesianWaypoint(Eigen::Isometry3d::Identity()), MoveInstructionType::FREESPACE));

  MotionPlanner::assignSolution(mi, joint_ids, values(), true);

  ASSERT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  const auto& cwp = mi.getWaypoint().as<CartesianWaypointPoly>();
  ASSERT_TRUE(cwp.hasSeed());
  EXPECT_EQ(cwp.getSeed().getJointIds(), joint_ids);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

#include <ruckig/input_parameter.hpp>
#include <ruckig/ruckig.hpp>
#include <ruckig/trajectory.hpp>

using namespace tesseract_planning;

// Initialize one-joint, 3 points exactly the same.
CompositeInstruction createRepeatedPointTrajectory()
{
  const int num = 3;
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
    swp.getPosition()[0] = 1;
    program.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  return program;
}

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  const int num = 10;
  const double max = 2.0;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
    swp.getPosition()[0] = i * max / num;
    program.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypointPoly swp{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  swp.getPosition()[0] = max;
  program.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

TEST(RuckigTrajectorySmoothingTest, Example)  // NOLINT
{
  // Create input parameters
  ruckig::InputParameter<3> input;
  input.current_position = { 0.0, 0.0, 0.5 };
  input.current_velocity = { 0.0, -2.2, -0.5 };
  input.current_acceleration = { 0.0, 2.5, -0.5 };

  input.target_position = { 5.0, -2.0, -3.5 };
  input.target_velocity = { 0.0, -0.5, -2.0 };
  input.target_acceleration = { 0.0, 0.0, 0.5 };

  input.max_velocity = { 3.0, 1.0, 3.0 };
  input.max_acceleration = { 3.0, 2.0, 1.0 };
  input.max_jerk = { 4.0, 3.0, 2.0 };

  // Set different constraints for negative direction
  input.min_velocity = { -2.0, -0.5, -3.0 };
  input.min_acceleration = { -2.0, -2.0, -2.0 };

  // We don't need to pass the control rate (cycle time) when using only offline features
  ruckig::Ruckig<3> otg;
  ruckig::Trajectory<3> trajectory;

  // Calculate the trajectory in an offline manner (outside of the control loop)
  ruckig::Result result = otg.calculate(input, trajectory);
  if (result == ruckig::Result::ErrorInvalidInput)
  {
    std::cout << "Invalid input!" << std::endl;
    return;
  }

  // Get duration of the trajectory
  std::cout << "Trajectory duration: " << trajectory.get_duration() << " [s]." << std::endl;

  double new_time{ 1.0 };

  // Then, we can calculate the kinematic state at a given time
  std::array<double, 3> new_position{}, new_velocity{}, new_acceleration{};
  trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);

  std::cout << "Position at time " << new_time << " [s]: " << new_position[0] << ", " << new_position[1] << ", "
            << new_position[2] << std::endl;

  // Get some info about the position extrema of the trajectory
  std::array<ruckig::PositionExtrema, 3> position_extrema = trajectory.get_position_extrema();
  std::cout << "Position extremas for DoF 4 are " << position_extrema[2].min << " (min) to " << position_extrema[2].max
            << " (max)" << std::endl;
}

TEST(RuckigTrajectorySmoothingTest, RuckigTrajectorySmoothingSolve)  // NOLINT
{
  IterativeSplineParameterization time_parameterization(false);
  CompositeInstruction program = createStraightTrajectory();
  std::vector<double> max_velocity = { 2.088, 2.082, 3.27, 3.6, 3.3, 3.078 };
  std::vector<double> max_acceleration = { 1, 1, 1, 1, 1, 1 };
  std::vector<double> max_jerk = { 1000, 1000, 1000, 1000, 1000, 1000 };
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(program);
  EXPECT_TRUE(time_parameterization.compute(*trajectory, max_velocity, max_acceleration));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.0);

  RuckigTrajectorySmoothing traj_smoothing;
  EXPECT_TRUE(traj_smoothing.compute(*trajectory, max_velocity, max_acceleration, max_jerk));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 8.0);
}

TEST(RuckigTrajectorySmoothingTest, RuckigTrajectorySmoothingRepeatedPointSolve)  // NOLINT
{
  IterativeSplineParameterization time_parameterization(true);
  CompositeInstruction program = createRepeatedPointTrajectory();
  std::vector<double> max_velocity = { 2.088, 2.082, 3.27, 3.6, 3.3, 3.078 };
  std::vector<double> max_acceleration = { 1, 1, 1, 1, 1, 1 };
  std::vector<double> max_jerk = { 1000, 1000, 1000, 1000, 1000, 1000 };
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(program);
  EXPECT_TRUE(time_parameterization.compute(*trajectory, max_velocity, max_acceleration));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 0.001);

  RuckigTrajectorySmoothing traj_smoothing;
  EXPECT_TRUE(traj_smoothing.compute(*trajectory, max_velocity, max_acceleration, max_jerk));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  return RUN_ALL_TESTS();
}

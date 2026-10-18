/**
 * @file totg_test_oracles.h
 * @brief Checks on a time-parameterized program shared by the TOTG tests
 *
 * @author Roelof Oomen
 * @date October 18, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_TOTG_TEST_ORACLES_H
#define TESSERACT_TIME_PARAMETERIZATION_TOTG_TEST_ORACLES_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>

namespace tesseract::time_parameterization::test_suite
{
/** @brief Expect every waypoint time to be finite and greater than the one before it, the last waypoint included */
inline void expectTimesStrictlyIncreasing(const tesseract::command_language::CompositeInstruction& program)
{
  using namespace tesseract::command_language;
  const auto moves = program.flatten(moveFilter);
  for (std::size_t i = 0; i < moves.size(); ++i)
  {
    const double t = moves[i].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime();
    EXPECT_TRUE(std::isfinite(t)) << "waypoint " << i;
    if (i > 0)
    {
      const double t_prev =
          moves[i - 1].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime();
      EXPECT_GT(t, t_prev) << "waypoint " << i;
    }
  }
}

/** @brief Expect every joint velocity of every waypoint to be within max_velocity, up to a relative tolerance */
inline void expectVelocitiesWithinLimits(const tesseract::command_language::CompositeInstruction& program,
                                         const Eigen::VectorXd& max_velocity,
                                         double relative_tolerance = 1e-6)
{
  using namespace tesseract::command_language;
  const auto moves = program.flatten(moveFilter);
  for (std::size_t i = 0; i < moves.size(); ++i)
  {
    const Eigen::VectorXd& v =
        moves[i].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getVelocity();
    ASSERT_EQ(v.size(), max_velocity.size()) << "waypoint " << i;
    for (Eigen::Index j = 0; j < v.size(); ++j)
      EXPECT_LE(std::abs(v[j]), max_velocity[j] * (1.0 + relative_tolerance)) << "waypoint " << i << " joint " << j;
  }
}
}  // namespace tesseract::time_parameterization::test_suite

#endif  // TESSERACT_TIME_PARAMETERIZATION_TOTG_TEST_ORACLES_H

/**
 * @file utils.cpp
 * @brief Time parameterization utils
 *
 * @author Matthew Powelson
 * @date January 26, 2021
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/core/utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

namespace tesseract_planning
{
void rescaleTimings(CompositeInstruction& program, std::vector<double> scalings)
{
  assert(program.size() == scalings.size());

  double prev_time_scaled = 0;
  double prev_time_original = 0;
  for (std::size_t sub_composite_idx = 0; sub_composite_idx < program.size(); sub_composite_idx++)
  {
    auto& sub = program[sub_composite_idx].as<CompositeInstruction>();
    for (auto& instruction : sub)
    {
      if (instruction.isMoveInstruction())
      {
        auto& move = instruction.as<MoveInstructionPoly>();
        if (move.getWaypoint().isStateWaypoint())
        {
          auto& state = move.getWaypoint().as<StateWaypointPoly>();

          double scaling_factor = scalings[sub_composite_idx];
          if (scaling_factor < 1e-6)
          {
            CONSOLE_BRIDGE_logWarn("Scaling factor is close to 0 (%f), defaulting to 1", scaling_factor);
            scaling_factor = 1.0;
          }

          state.setVelocity(state.getVelocity() * scaling_factor);
          state.setAcceleration(state.getAcceleration() * scaling_factor * scaling_factor);
          state.setEffort(state.getEffort() * scaling_factor * scaling_factor);

          double temp = state.getTime();
          // scale dt and add to previous time
          state.setTime(prev_time_scaled + ((temp - prev_time_original) / scaling_factor));
          prev_time_scaled = state.getTime();
          prev_time_original = temp;
        }
      }
    }
  }
}
}  // namespace tesseract_planning

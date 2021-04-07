/**
 * @file utils.h
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_UTILS_H
#define TESSERACT_TIME_PARAMETERIZATION_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

namespace tesseract_planning
{
/**
 * @brief Rescale the sub-composites of a program linearly
 * @param program Input to be scaled. Each element should be a CompositeInstruction
 * @param scalings Scaling to apply to each sub-composite. Should be length program.size(). Example: 0.5 will be half
 * speed
 * @return True if successful
 */
void RescaleTimings(CompositeInstruction& program, std::vector<double> scalings)
{
  assert(program.size() == scalings.size());

  double prev_time_scaled = 0;
  double prev_time_original = 0;
  for (std::size_t sub_composite_idx = 0; sub_composite_idx < program.size(); sub_composite_idx++)
  {
    CompositeInstruction& sub = program[sub_composite_idx].as<CompositeInstruction>();
    for (std::size_t move_idx = 0; move_idx < sub.size(); move_idx++)
    {
      if (isMoveInstruction(sub.at(move_idx)))
      {
        auto& move = sub.at(move_idx).as<MoveInstruction>();
        if (isStateWaypoint(move.getWaypoint()))
        {
          auto& state = move.getWaypoint().as<StateWaypoint>();

          double scaling_factor = scalings[sub_composite_idx];
          if (scaling_factor < 1e-6)
          {
            CONSOLE_BRIDGE_logWarn("Scaling factor is close to 0 (%f), defaulting to 1", scaling_factor);
            scaling_factor = 1.0;
          }

          state.velocity = state.velocity * scaling_factor;
          state.acceleration = state.acceleration * scaling_factor * scaling_factor;
          state.effort = state.effort * scaling_factor * scaling_factor;

          double temp = state.time;
          state.time = prev_time_scaled +
                       (state.time - prev_time_original) / scaling_factor;  // scale dt and add to previous time
          prev_time_scaled = state.time;
          prev_time_original = temp;
        }
      }
    }
  }
}
}  // namespace tesseract_planning

#endif

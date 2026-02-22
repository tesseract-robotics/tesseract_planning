/**
 * @file utils.h
 * @brief Time parameterization utils
 *
 * @author Matthew Powelson
 * @date January 26, 2021
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
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/fwd.h>

namespace tesseract::time_parameterization
{
/**
 * @brief Rescale the sub-composites of a program linearly
 * @param program Input to be scaled. Each element should be a CompositeInstruction
 * @param scalings Scaling to apply to each sub-composite. Should be length program.size(). Example: 0.5 will be half
 * speed
 * @return True if successful
 */
void rescaleTimings(tesseract::command_language::CompositeInstruction& program, std::vector<double> scalings);
}  // namespace tesseract::time_parameterization

#endif

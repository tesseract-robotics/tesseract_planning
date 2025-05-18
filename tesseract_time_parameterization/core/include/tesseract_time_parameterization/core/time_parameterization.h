/**
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H

#include <tesseract_common/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/composite_instruction.h>

#include <string>

namespace tesseract_planning
{
/** @brief A generic container that the time parameterization classes use */
class TimeParameterization
{
public:
  TimeParameterization() = default;
  TimeParameterization(std::string name);
  virtual ~TimeParameterization() = default;
  TimeParameterization(const TimeParameterization&) = delete;
  TimeParameterization& operator=(const TimeParameterization&) = delete;
  TimeParameterization(TimeParameterization&&) = delete;
  TimeParameterization& operator=(TimeParameterization&&) = delete;

  /**
   * @brief Get the name of this time parameterization
   * @details This is also used as the namespace for the profiles in the profile dictionary
   */
  const std::string& getName() const;

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param composite_instruction The composite instruction
   * @param env The environment
   * @param profiles The profile dictionary
   * @return True if successful, otherwise false
   */
  virtual bool compute(CompositeInstruction& composite_instruction,
                       const tesseract_environment::Environment& env,
                       const tesseract_common::ProfileDictionary& profiles) const = 0;

protected:
  std::string name_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H

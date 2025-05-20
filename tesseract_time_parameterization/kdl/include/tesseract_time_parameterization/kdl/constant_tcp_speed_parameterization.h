/**
 * @file contant_tcp_speed_parameterization.h
 * @brief Constant Tool Center Point (TCP) speed time parameterization
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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

#ifndef TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_H

#include <tesseract_time_parameterization/core/fwd.h>
#include <tesseract_time_parameterization/core/time_parameterization.h>

namespace tesseract_planning
{
/**
 * @brief Time parameterization based on trapazodal profile leveraging KDL
 * @details This was migrated from scan_n_plan_workshop but believe further improvements are needed.
 * - Does this approach ensure it does not violate the kinematic limits?
 * - What base frame should be used for calculation?
 *   - I believe we should leverage the composite base frame and transform
 */
class ConstantTCPSpeedParameterization : public TimeParameterization
{
public:
  explicit ConstantTCPSpeedParameterization(std::string name);
  ~ConstantTCPSpeedParameterization() override = default;
  ConstantTCPSpeedParameterization(const ConstantTCPSpeedParameterization&) = delete;
  ConstantTCPSpeedParameterization& operator=(const ConstantTCPSpeedParameterization&) = delete;
  ConstantTCPSpeedParameterization(ConstantTCPSpeedParameterization&&) = delete;
  ConstantTCPSpeedParameterization& operator=(ConstantTCPSpeedParameterization&&) = delete;

  bool compute(CompositeInstruction& composite_instruction,
               const tesseract_environment::Environment& env,
               const tesseract_common::ProfileDictionary& profiles) const override;
};

}  // namespace tesseract_planning

#endif

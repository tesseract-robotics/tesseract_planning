/**
 * @file ruckig_trajectory_smoothing.h
 * @brief Leveraging Ruckig to smooth trajectory
 *
 * @author Levi Armstrong
 * @date July 27, 2022
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

#ifndef TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H
#define TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/core/fwd.h>
#include <tesseract_time_parameterization/core/time_parameterization.h>

namespace tesseract_planning
{
class RuckigTrajectorySmoothing : public TimeParameterization
{
public:
  RuckigTrajectorySmoothing(std::string name);
  ~RuckigTrajectorySmoothing() override = default;
  RuckigTrajectorySmoothing(const RuckigTrajectorySmoothing&) = delete;
  RuckigTrajectorySmoothing& operator=(const RuckigTrajectorySmoothing&) = delete;
  RuckigTrajectorySmoothing(RuckigTrajectorySmoothing&&) = delete;
  RuckigTrajectorySmoothing& operator=(RuckigTrajectorySmoothing&&) = delete;

  bool compute(CompositeInstruction& composite_instruction,
               const tesseract_environment::Environment& env,
               const tesseract_common::ProfileDictionary& profiles) const override;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H

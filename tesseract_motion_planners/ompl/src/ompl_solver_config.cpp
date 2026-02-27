/**
 * @file ompl_solver_config.cpp
 * @brief Tesseract OMPL solver config
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
bool OMPLSolverConfig::operator==(const OMPLSolverConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract::common::almostEqualRelativeAndAbs(planning_time, rhs.planning_time, max_diff);
  equal &= (max_solutions == rhs.max_solutions);
  equal &= (simplify == rhs.simplify);
  equal &= (optimize == rhs.optimize);
  equal &= (planners.size() == rhs.planners.size());

  return equal;
}
bool OMPLSolverConfig::operator!=(const OMPLSolverConfig& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::motion_planners

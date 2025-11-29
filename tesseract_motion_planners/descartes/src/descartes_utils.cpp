/**
 * @file descartes_utils.h
 * @brief
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

#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                  const Eigen::Vector3d& axis,
                                                  double resolution,
                                                  double minimum,
                                                  double maximum)
{
  tesseract_common::VectorIsometry3d samples;
  auto cnt = static_cast<int>(std::ceil((maximum - minimum) / resolution)) + 1;
  samples.reserve(static_cast<size_t>(cnt));

  double angle{ minimum };
  while (angle < maximum || tesseract_common::almostEqualRelativeAndAbs(angle, maximum))
  {
    Eigen::Isometry3d p = tool_pose * Eigen::AngleAxisd(angle, axis);
    samples.push_back(p);
    angle += resolution;
  }

  return samples;
}

tesseract_common::VectorIsometry3d
sampleToolXAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum)
{
  return sampleToolAxis(tool_pose, Eigen::Vector3d::UnitX(), resolution, minimum, maximum);  // NOLINT
}

tesseract_common::VectorIsometry3d
sampleToolYAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum)
{
  return sampleToolAxis(tool_pose, Eigen::Vector3d::UnitY(), resolution, minimum, maximum);  // NOLINT
}

tesseract_common::VectorIsometry3d
sampleToolZAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum)
{
  return sampleToolAxis(tool_pose, Eigen::Vector3d::UnitZ(), resolution, minimum, maximum);  // NOLINT
}

tesseract_common::VectorIsometry3d sampleFixed(const Eigen::Isometry3d& tool_pose)
{
  return tesseract_common::VectorIsometry3d({ tool_pose });
}
}  // namespace tesseract_planning

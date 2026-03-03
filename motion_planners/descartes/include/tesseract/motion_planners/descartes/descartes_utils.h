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
#ifndef TESSERACT_PLANNING_DESCARTES_UTILS_H
#define TESSERACT_PLANNING_DESCARTES_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/eigen_types.h>

namespace tesseract::motion_planners
{
using PoseSamplerFn = std::function<tesseract::common::VectorIsometry3d(const Eigen::Isometry3d& tool_pose)>;

/**
 * @brief Given a tool pose create samples from [minimum, maximum] around the provided axis.
 * @param tool_pose Tool pose to be sampled
 * @param axis The axis to sample around
 * @param resolution The resolution to sample at
 * @param minimum The minimum angle to start sampling at
 * @param maximum The maximum angle to stop sampling at
 * @return A vector of tool poses
 */
tesseract::common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                   const Eigen::Vector3d& axis,
                                                   double resolution,
                                                   double minimum,
                                                   double maximum);

/**
 * @brief Given a tool pose create samples from [minimum, maximum] around the x axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param minimum The minimum angle to start sampling at
 * @param maximum The maximum angle to stop sampling at
 * @return A vector of tool poses
 */
tesseract::common::VectorIsometry3d
sampleToolXAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum);

/**
 * @brief Given a tool pose create samples from [minimum, maximum] around the y axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param minimum The minimum angle to start sampling at
 * @param maximum The maximum angle to stop sampling at
 * @return A vector of tool poses
 */
tesseract::common::VectorIsometry3d
sampleToolYAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum);

/**
 * @brief Given a tool pose create samples from [minimum, maximum] around the z axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param minimum The minimum angle to start sampling at
 * @param maximum The maximum angle to stop sampling at
 * @return A vector of tool poses
 */
tesseract::common::VectorIsometry3d
sampleToolZAxis(const Eigen::Isometry3d& tool_pose, double resolution, double minimum, double maximum);

/**
 * @brief This is the default sample with if a fixed pose sampler
 * @param tool_pose Tool pose to be sampled
 * @return A vector with a single pose that was provided as input to function
 */
tesseract::common::VectorIsometry3d sampleFixed(const Eigen::Isometry3d& tool_pose);

}  // namespace tesseract::motion_planners
#endif  // TESSERACT_PLANNING_DESCARTES_UTILS_H

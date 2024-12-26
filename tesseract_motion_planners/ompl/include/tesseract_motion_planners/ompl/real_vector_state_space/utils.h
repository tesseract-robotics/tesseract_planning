/**
 * @file utils.h
 * @brief Tesseract OMPL real vector state space utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_UTILS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_types.h>

namespace ompl::base
{
class State;
}  // namespace ompl::base

namespace ompl::geometric
{
class PathGeometric;
}

namespace tesseract_planning
{
/**
 * @brief Converts an OMPL state into a vector of doubles
 * @param s1 OMPL state
 * @param dimension Size of the state (e.g., number of joints)
 * @return
 */
Eigen::Map<Eigen::VectorXd> fromRealVectorStateSpace(const ompl::base::State* s1, unsigned dimension);

/**
 * @brief Converts
 * @param path
 * @return
 */
tesseract_common::TrajArray fromRealVectorStateSpace(const ompl::geometric::PathGeometric& path);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_UTILS_H

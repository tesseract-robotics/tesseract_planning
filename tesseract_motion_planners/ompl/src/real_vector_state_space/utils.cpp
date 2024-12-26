/**
 * @file utils.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>

// #ifndef OMPL_LESS_1_4_0
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
// #endif

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/real_vector_state_space/utils.h>

#include <tesseract_common/types.h>

namespace tesseract_planning
{
Eigen::Map<Eigen::VectorXd> fromRealVectorStateSpace(const ompl::base::State* s1, unsigned dimension)
{
  assert(dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(s1) != nullptr);
  const auto* s = s1->template as<ompl::base::RealVectorStateSpace::StateType>();
  return Eigen::Map<Eigen::VectorXd>{ s->values, dimension };
}

tesseract_common::TrajArray fromRealVectorStateSpace(const ompl::geometric::PathGeometric& path)
{
  const auto n_points = static_cast<long>(path.getStateCount());
  const auto dof = static_cast<unsigned>(path.getSpaceInformation()->getStateDimension());

  tesseract_common::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
    result.row(i) = fromRealVectorStateSpace(path.getState(static_cast<unsigned>(i)), dof);

  return result;
}

}  // namespace tesseract_planning

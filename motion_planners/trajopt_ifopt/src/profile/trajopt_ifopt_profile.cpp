/**
 * @file trajopt_ifopt_profile.cpp
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <utility>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace tesseract::motion_planners
{
TrajOptIfoptMoveProfile::TrajOptIfoptMoveProfile() : Profile(createKey<TrajOptIfoptMoveProfile>()) {}

TrajOptIfoptCompositeProfile::TrajOptIfoptCompositeProfile() : Profile(createKey<TrajOptIfoptCompositeProfile>()) {}

TrajOptIfoptSolverProfile::TrajOptIfoptSolverProfile() : Profile(createKey<TrajOptIfoptSolverProfile>()) {}

std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>> TrajOptIfoptSolverProfile::createOptimizationCallbacks() const
{
  return callbacks;
}

std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver>
TrajOptIfoptSolverProfile::createSolver(std::shared_ptr<trajopt_sqp::QPSolver> qp_solver, bool verbose) const
{
  auto solver = std::make_unique<trajopt_sqp::TrustRegionSQPSolver>(std::move(qp_solver));
  solver->params = opt_params;
  solver->verbose = verbose;

  for (const trajopt_sqp::SQPCallback::Ptr& callback : createOptimizationCallbacks())
    solver->registerCallback(callback);

  return solver;
}

}  // namespace tesseract::motion_planners

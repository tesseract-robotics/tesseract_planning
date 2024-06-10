/**
 * @file trajopt_ifopt_problem.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <OsqpEigen/Settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>

namespace tesseract_planning
{
TrajOptIfoptProblem::TrajOptIfoptProblem()
{
  convex_solver_settings = std::make_unique<OsqpEigen::Settings>();
  convex_solver_settings->setVerbosity(false);
  convex_solver_settings->setWarmStart(true);
  convex_solver_settings->setPolish(true);
  convex_solver_settings->setAdaptiveRho(true);
  convex_solver_settings->setMaxIteration(8192);
  convex_solver_settings->setAbsoluteTolerance(1e-4);
  convex_solver_settings->setRelativeTolerance(1e-6);
}

}  // namespace tesseract_planning

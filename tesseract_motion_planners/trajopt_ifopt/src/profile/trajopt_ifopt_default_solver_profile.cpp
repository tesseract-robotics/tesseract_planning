/**
 * @file trajopt_default_solver_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date December 13, 2020
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
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>

namespace tesseract_planning
{

TrajOptIfoptDefaultSolverProfile::TrajOptIfoptDefaultSolverProfile()
{
  osqp_set_default_settings(&convex_solver_settings);
  convex_solver_settings.verbose = 0;
  convex_solver_settings.warm_start = 1;
  convex_solver_settings.polish = 1;
  convex_solver_settings.adaptive_rho = 1;
  convex_solver_settings.max_iter = 8192;
  convex_solver_settings.eps_abs = 1e-4;
  convex_solver_settings.eps_rel = 1e-6;
}

void TrajOptIfoptDefaultSolverProfile::apply(TrajOptIfoptProblem& problem) const
{
  problem.convex_solver_settings = convex_solver_settings;
  problem.opt_info = opt_info;
  problem.callbacks = callbacks;
}

tinyxml2::XMLElement* TrajOptIfoptDefaultSolverProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultSolverProfile::toXML is not implemented!");
}

}  // namespace tesseract_planning

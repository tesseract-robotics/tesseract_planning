/**
 * @file trajopt_default_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_SOLVER_PROFILE_H

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <trajopt_sqp/sqp_callback.h>

namespace tesseract_planning
{
/** @brief The contains the default solver parameters available for setting up TrajOpt */
class TrajOptIfoptDefaultSolverProfile : public TrajOptIfoptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptDefaultSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptDefaultSolverProfile>;

  TrajOptIfoptDefaultSolverProfile() = default;
  ~TrajOptIfoptDefaultSolverProfile() override = default;
  TrajOptIfoptDefaultSolverProfile(const TrajOptIfoptDefaultSolverProfile&) = default;
  TrajOptIfoptDefaultSolverProfile& operator=(const TrajOptIfoptDefaultSolverProfile&) = default;
  TrajOptIfoptDefaultSolverProfile(TrajOptIfoptDefaultSolverProfile&&) = default;
  TrajOptIfoptDefaultSolverProfile& operator=(TrajOptIfoptDefaultSolverProfile&&) = default;

  /** @brief Optimization paramters */
  trajopt_sqp::SQPParameters opt_info;

  /** @brief Optimization callbacks */
  std::vector<trajopt_sqp::SQPCallback::Ptr> callbacks;

  void apply(TrajOptIfoptProblem& problem) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_SOLVER_PROFILE_H

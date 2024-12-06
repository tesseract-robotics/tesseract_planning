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
#include <OsqpEigen/Settings.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

namespace tesseract_planning
{
TrajOptIfoptDefaultSolverProfile::TrajOptIfoptDefaultSolverProfile()
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

TrajOptIfoptDefaultSolverProfile::~TrajOptIfoptDefaultSolverProfile() = default;

void TrajOptIfoptDefaultSolverProfile::apply(TrajOptIfoptProblem& problem) const
{
  copyOSQPEigenSettings(*problem.convex_solver_settings, *convex_solver_settings);
  problem.opt_info = opt_info;
  problem.callbacks = callbacks;
}

tinyxml2::XMLElement* TrajOptIfoptDefaultSolverProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultSolverProfile::toXML is not implemented!");
}

template <class Archive>
void TrajOptIfoptDefaultSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptSolverProfile);
  /** @todo FIX */
  // ar& BOOST_SERIALIZATION_NVP(convex_solver_settings);
  // ar& BOOST_SERIALIZATION_NVP(opt_info);
  // ar& BOOST_SERIALIZATION_NVP(callbacks);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptDefaultSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptDefaultSolverProfile)

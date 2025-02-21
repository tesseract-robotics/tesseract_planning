/**
 * @file trajopt_osqp_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H

#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <trajopt_sco/osqp_interface.hpp>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, OSQPSettings& settings, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, sco::BasicTrustRegionSQPParameters& params, const unsigned int version);  // NOLINT

}  // namespace boost::serialization

namespace tesseract_planning
{
/** @brief The contains the OSQP Solver and Trust Region Parameters available for setting up TrajOpt */
class TrajOptOSQPSolverProfile : public TrajOptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptOSQPSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptOSQPSolverProfile>;

  TrajOptOSQPSolverProfile();

  OSQPSettings settings{};

  bool update_workspace{ false };

  sco::ModelType getSolverType() const override;

  std::unique_ptr<sco::ModelConfig> createSolverConfig() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptOSQPSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H

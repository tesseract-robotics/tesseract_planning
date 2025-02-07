/**
 * @file iterative_spline_parameterization_profile.h
 * @brief Profile for iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILE_H
#define TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/profile.h>

namespace tesseract_planning
{
struct IterativeSplineParameterizationProfile : public Profile
{
  using Ptr = std::shared_ptr<IterativeSplineParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const IterativeSplineParameterizationProfile>;

  IterativeSplineParameterizationProfile();
  IterativeSplineParameterizationProfile(double max_velocity_scaling_factor, double max_acceleration_scaling_factor);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor{ 1.0 };

  /**
   * @brief minimum_time_delta The smallest-allowable difference between timestamps of
   * consecutive trajectory points. Passed to the solver.
   */
  double minimum_time_delta{ std::numeric_limits<double>::epsilon() };

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::IterativeSplineParameterizationProfile)

#endif  // TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILE_H

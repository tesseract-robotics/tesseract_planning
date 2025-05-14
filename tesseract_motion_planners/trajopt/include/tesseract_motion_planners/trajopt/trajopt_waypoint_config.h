/**
 * @file trajopt_waypoint_config.h
 * @brief TrajOpt waypoint configuration settings
 *
 * @author Tyler Marr
 * @date November 2, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_WAYPOINT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_WAYPOINT_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
/**
 * @brief Config settings for cartesian waypoints
 */
struct TrajOptCartesianWaypointConfig
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  TrajOptCartesianWaypointConfig() = default;

  /** @brief If true, a cost/constraint term will be added to the problem. Default: true*/
  bool enabled{ true };

  /** @brief If true, will override existing waypoint tolerance with described tolerance here. Default: false
   * This is useful if you want to have a smaller tolerance for the cost than the constraint.*/
  bool use_tolerance_override{ false };

  /** @brief Distance below waypoint that is allowed. Should be size = 6 or 1, if size = 1 the value is replicated.
   * First 3 elements are dx, dy, dz. The last 3 elements are angle axis error allowed:
   * (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd lower_tolerance{ Eigen::VectorXd::Zero(6) };
  /** @brief Distance above waypoint that is allowed. Should be size = 6 or 1, if size = 1 the value is replicated.
   * First 3 elements are dx, dy, dz. The last 3 elements are angle axis error allowed:
   * (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd upper_tolerance{ Eigen::VectorXd::Zero(6) };

  /** @brief coefficients corresponsing to dx, dy, dz, rx, ry, rz.
   * Should be size = 6 or 1, if size = 1 the value is replicated. */
  Eigen::VectorXd coeff{ Eigen::VectorXd::Constant(6, 5) };

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

/**
 * @brief Config settings for joint waypoints.
 */
struct TrajOptJointWaypointConfig
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  TrajOptJointWaypointConfig() = default;

  /** @brief If true, a cost/constraint term will be added to the problem. Default: true*/
  bool enabled{ true };

  /** @brief If true, will override existing waypoint tolerance with described tolerance here. Default: false
   * This is useful if you want to have a smaller tolerance for the cost than the constraint.*/
  bool use_tolerance_override{ false };

  /** @brief Distance below waypoint that is allowed. Should be size of joints in a joint state*/
  Eigen::VectorXd lower_tolerance;
  /** @brief Distance above waypoint that is allowed. Should be size of joints in a joint state*/
  Eigen::VectorXd upper_tolerance;

  /** @brief coefficients corresponsing to joint values*/
  Eigen::VectorXd coeff{ Eigen::VectorXd::Constant(1, 1, 5) };

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptCartesianWaypointConfig)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptJointWaypointConfig)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_WAYPOINT_CONFIG_H

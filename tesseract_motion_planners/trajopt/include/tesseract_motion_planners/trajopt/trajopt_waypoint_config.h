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
#include <tesseract_common/utils.h>

namespace tinyxml2
{
// NOLINTNEXTLINE(cppcoreguidelines-virtual-class-destructor)
class XMLElement;
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract_planning
{
/**
 * @brief Config settings for cartesian waypoints
 */
struct CartesianWaypointConfig
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  CartesianWaypointConfig() = default;
  CartesianWaypointConfig(const tinyxml2::XMLElement& xml_element);

  /** @brief If true, a cost/constraint term will be added to the problem. Default: true*/
  bool enabled{ true };

  /** @brief If true, will override existing waypoint tolerance with described tolerance here. Default: false
   * This is useful if you want to have a smaller tolerance for the cost than the constraint.*/
  bool use_tolerance_override{ false };

  /** @brief Distance below waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::Matrix<double, 6, 1> lower_tolerance{ Eigen::VectorXd::Zero(6) };
  /** @brief Distance above waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*/
  Eigen::Matrix<double, 6, 1> upper_tolerance{ Eigen::VectorXd::Zero(6) };

  /** @brief coefficients corresponsing to dx, dy, dz, rx, ry, rz*/
  Eigen::Matrix<double, 6, 1> coeff{ Eigen::VectorXd::Constant(6, 5) };

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;
};

/**
 * @brief Config settings for joint waypoints.
 */
struct JointWaypointConfig
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  JointWaypointConfig() = default;
  JointWaypointConfig(const tinyxml2::XMLElement& xml_element);

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

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_WAYPOINT_CONFIG_H

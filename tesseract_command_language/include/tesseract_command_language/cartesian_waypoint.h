/**
 * @file cartesian_waypoint.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_common/joint_state.h>

namespace tesseract_planning
{
class CartesianWaypoint final : public CartesianWaypointInterface
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  CartesianWaypoint() = default;
  CartesianWaypoint(const Eigen::Isometry3d& transform);
  CartesianWaypoint(const Eigen::Isometry3d& transform,
                    const Eigen::VectorXd& lower_tol,
                    const Eigen::VectorXd& upper_tol);

  // Waypoint
  void setName(const std::string& name) override final;
  const std::string& getName() const override final;
  void print(const std::string& prefix = "") const override final;

  // Cartesian Waypoint
  void setTransform(const Eigen::Isometry3d& transform) override final;
  Eigen::Isometry3d& getTransform() override final;
  const Eigen::Isometry3d& getTransform() const override final;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol) override final;
  Eigen::VectorXd& getUpperTolerance() override final;
  const Eigen::VectorXd& getUpperTolerance() const override final;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol) override final;
  Eigen::VectorXd& getLowerTolerance() override final;
  const Eigen::VectorXd& getLowerTolerance() const override final;

  void setSeed(const tesseract_common::JointState& seed) override final;
  tesseract_common::JointState& getSeed() override final;
  const tesseract_common::JointState& getSeed() const override final;

  std::unique_ptr<CartesianWaypointInterface> clone() const override final;

private:
  /** @brief The name of the waypoint */
  std::string name_;
  /** @brief The Cartesian Waypoint */
  Eigen::Isometry3d transform_{ Eigen::Isometry3d::Identity() };
  /** @brief Distance below waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd lower_tolerance_;
  /** @brief Distance above waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*/
  Eigen::VectorXd upper_tolerance_;

  /**
   * @brief Seed associated with this Cartesian waypoint
   * @details The waypoint seed can be used for purposes like:
   *   - providing a joint state seed to an IK solver
   *   - providing a seed to the IK solver with modified limits using the tolerances
   *   - providing a joint state to be used by a motion planner for interpolation to avoid performing IK
   */
  tesseract_common::JointState seed_;

  bool equals(const CartesianWaypointInterface& other) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::CartesianWaypoint)
BOOST_CLASS_TRACKING(tesseract_planning::CartesianWaypoint, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H

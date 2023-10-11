/**
 * @file joint_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class JointWaypoint
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  JointWaypoint() = default;
  JointWaypoint(std::vector<std::string> names, const Eigen::VectorXd& position, bool is_constrained = true);
  JointWaypoint(std::vector<std::string> names,
                const Eigen::VectorXd& position,
                const Eigen::VectorXd& lower_tol,
                const Eigen::VectorXd& upper_tol);
  JointWaypoint(std::initializer_list<std::string> names,
                std::initializer_list<double> position,
                bool is_constrained = true);
  JointWaypoint(std::initializer_list<std::string> names,
                std::initializer_list<double> position,
                std::initializer_list<double> lower_tol,
                std::initializer_list<double> upper_tol);

  void setNames(const std::vector<std::string>& names);
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  Eigen::VectorXd& getUpperTolerance();
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  Eigen::VectorXd& getLowerTolerance();
  const Eigen::VectorXd& getLowerTolerance() const;

  void setIsConstrained(bool value);
  bool isConstrained() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

  bool operator==(const JointWaypoint& rhs) const;
  bool operator!=(const JointWaypoint& rhs) const;

protected:
  /** @brief The name of the waypoint */
  std::string name_;
  /** @brief The names of the joints */
  std::vector<std::string> names_;
  /** @brief The position of the joints */
  Eigen::VectorXd position_;
  /** @brief Joint distance below position that is allowed. Each element should be <= 0 */
  Eigen::VectorXd lower_tolerance_;
  /** @brief Joint distance above position that is allowed. Each element should be >= 0 */
  Eigen::VectorXd upper_tolerance_;
  /** @brief Indicates if it is constrained joint state */
  bool is_constrained_{ false };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(tesseract_planning, JointWaypoint)

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

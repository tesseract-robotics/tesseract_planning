/**
 * @file joint_waypoint.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <utility>
#include <vector>
#include <type_traits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/command_language/poly/joint_waypoint_poly.h>
#include <tesseract/common/types.h>

namespace tesseract::command_language
{
class JointWaypoint;

template <class Archive>
void save(Archive& ar, const JointWaypoint& obj);
template <class Archive>
void load(Archive& ar, JointWaypoint& obj);

class JointWaypoint final : public JointWaypointInterface
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  JointWaypoint() = default;

  // SFINAE-guarded string constructors (legacy/convenience). Names are hashed into JointIds via toIds.
  template <typename T,
            std::enable_if_t<std::is_same_v<std::decay_t<T>, std::vector<std::string>>, int> = 0>
  JointWaypoint(const T& names, Eigen::VectorXd position, bool is_constrained = true)
    : joint_ids_(tesseract::common::toIds<tesseract::common::JointId>(names))
    , position_(std::move(position))
    , is_constrained_(is_constrained)
  {
    if (static_cast<Eigen::Index>(joint_ids_.size()) != position_.size())
      throw std::runtime_error("JointWaypoint: parameters are not the same size!");
  }

  template <typename T,
            std::enable_if_t<std::is_same_v<std::decay_t<T>, std::vector<std::string>>, int> = 0>
  JointWaypoint(const T& names, Eigen::VectorXd position, Eigen::VectorXd lower_tol, Eigen::VectorXd upper_tol)
    : joint_ids_(tesseract::common::toIds<tesseract::common::JointId>(names))
    , position_(std::move(position))
    , lower_tolerance_(std::move(lower_tol))
    , upper_tolerance_(std::move(upper_tol))
    , is_constrained_(true)
  {
    if (static_cast<Eigen::Index>(joint_ids_.size()) != position_.size() ||
        position_.size() != lower_tolerance_.size() || position_.size() != upper_tolerance_.size())
      throw std::runtime_error("JointWaypoint: parameters are not the same size!");
  }

  // JointId constructors (preferred path; no name->id hashing)
  JointWaypoint(std::vector<tesseract::common::JointId> joint_ids,
                Eigen::VectorXd position,
                bool is_constrained = true);
  JointWaypoint(std::vector<tesseract::common::JointId> joint_ids,
                Eigen::VectorXd position,
                Eigen::VectorXd lower_tol,
                Eigen::VectorXd upper_tol);
  JointWaypoint(std::initializer_list<std::string> names,
                std::initializer_list<double> position,
                bool is_constrained = true);
  JointWaypoint(std::initializer_list<std::string> names,
                std::initializer_list<double> position,
                std::initializer_list<double> lower_tol,
                std::initializer_list<double> upper_tol);

  // Waypoint
  void setName(const std::string& name) override final;
  const std::string& getName() const override final;
  void print(const std::string& prefix = "") const override final;

  // Joint Waypoint
  void setNames(const std::vector<std::string>& names) override final;
  std::vector<std::string> getNames() const override final;

  void setJointIds(const std::vector<tesseract::common::JointId>& ids) override final;
  const std::vector<tesseract::common::JointId>& getJointIds() const override final;
  std::vector<tesseract::common::JointId>& getJointIds() override final;

  void setPosition(const Eigen::VectorXd& position) override final;
  Eigen::VectorXd& getPosition() override final;
  const Eigen::VectorXd& getPosition() const override final;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol) override final;
  Eigen::VectorXd& getUpperTolerance() override final;
  const Eigen::VectorXd& getUpperTolerance() const override final;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol) override final;
  Eigen::VectorXd& getLowerTolerance() override final;
  const Eigen::VectorXd& getLowerTolerance() const override final;

  void setIsConstrained(bool value) override final;
  bool isConstrained() const override final;

  std::unique_ptr<JointWaypointInterface> clone() const override final;

private:
  /** @brief The name of the waypoint */
  std::string name_;
  /** @brief The joint IDs */
  std::vector<tesseract::common::JointId> joint_ids_;
  /** @brief The position of the joints */
  Eigen::VectorXd position_;
  /** @brief Joint distance below position that is allowed. Each element should be <= 0 */
  Eigen::VectorXd lower_tolerance_;
  /** @brief Joint distance above position that is allowed. Each element should be >= 0 */
  Eigen::VectorXd upper_tolerance_;
  /** @brief Indicates if it is constrained joint state */
  bool is_constrained_{ false };

  bool equals(const JointWaypointInterface& other) const override final;

  template <class Archive>
  friend void ::tesseract::command_language::save(Archive& ar, const JointWaypoint& obj);
  template <class Archive>
  friend void ::tesseract::command_language::load(Archive& ar, JointWaypoint& obj);
};
}  // namespace tesseract::command_language

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

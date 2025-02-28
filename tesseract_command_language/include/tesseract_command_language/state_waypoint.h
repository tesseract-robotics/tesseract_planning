/**
 * @file state_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/state_waypoint_poly.h>

namespace tesseract_planning
{
class StateWaypoint final : public StateWaypointInterface
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  StateWaypoint() = default;
  StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position);
  StateWaypoint(const std::vector<std::string>& names,
                const Eigen::VectorXd& position,
                const Eigen::VectorXd& velocity,
                const Eigen::VectorXd& acceleration,
                double time);

  StateWaypoint(std::initializer_list<std::string> names, std::initializer_list<double> position);
  StateWaypoint(std::initializer_list<std::string> names,
                std::initializer_list<double> position,
                std::initializer_list<double> velocity,
                std::initializer_list<double> acceleration,
                double time);

  // Waypoint
  void setName(const std::string& name) override final;
  const std::string& getName() const override final;
  void print(const std::string& prefix = "") const override final;

  // State Waypoint
  void setNames(const std::vector<std::string>& names) override final;
  std::vector<std::string>& getNames() override final;
  const std::vector<std::string>& getNames() const override final;

  void setPosition(const Eigen::VectorXd& position) override final;
  Eigen::VectorXd& getPosition() override final;
  const Eigen::VectorXd& getPosition() const override final;

  void setVelocity(const Eigen::VectorXd& velocity) override final;
  Eigen::VectorXd& getVelocity() override final;
  const Eigen::VectorXd& getVelocity() const override final;

  void setAcceleration(const Eigen::VectorXd& acceleration) override final;
  Eigen::VectorXd& getAcceleration() override final;
  const Eigen::VectorXd& getAcceleration() const override final;

  void setEffort(const Eigen::VectorXd& effort) override final;
  Eigen::VectorXd& getEffort() override final;
  const Eigen::VectorXd& getEffort() const override final;

  void setTime(double time) override final;
  double getTime() const override final;

  std::unique_ptr<StateWaypointInterface> clone() const override final;

private:
  /** @brief The name of the waypoint */
  std::string name_;
  /** @brief The joint corresponding to the position vector. */
  std::vector<std::string> joint_names_;
  /** @brief The joint position at the waypoint */
  Eigen::VectorXd position_;
  /** @brief The velocity at the waypoint (optional) */
  Eigen::VectorXd velocity_;
  /** @brief The Acceleration at the waypoint (optional) */
  Eigen::VectorXd acceleration_;
  /** @brief The Effort at the waypoint (optional) */
  Eigen::VectorXd effort_;
  /** @brief The Time from start at the waypoint (optional) */
  double time_{ 0 };

  bool equals(const StateWaypointInterface& other) const override final;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::StateWaypoint)
BOOST_CLASS_TRACKING(tesseract_planning::StateWaypoint, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

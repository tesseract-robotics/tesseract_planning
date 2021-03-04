/**
 * @file trajectory_container.h
 * @brief Creates an interface for contaning different trajectory data structures
 *
 * @author Levi Armstrong
 * @date March 3, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_TRAJECTORY_CONTAINER_H
#define TESSERACT_TIME_PARAMETERIZATION_TRAJECTORY_CONTAINER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/** @brief A generic container that the time parameterization classes use */
class TrajectoryContainer
{
public:
  using Ptr = std::shared_ptr<TrajectoryContainer>;
  using ConstPtr = std::shared_ptr<const TrajectoryContainer>;

  virtual ~TrajectoryContainer() = default;

  /**
   * @brief Get the position data at a given index
   * @param i The index to extract position data
   * @return The position data
   */
  virtual const Eigen::VectorXd& getPosition(Eigen::Index i) const = 0;

  /**
   * @brief Get the velocity data at a given index
   * @param i The index to extract velocity data
   * @return The velocity data
   */
  virtual const Eigen::VectorXd& getVelocity(Eigen::Index i) const = 0;

  /**
   * @brief Get the acceleration data at a given index
   * @param i The index to extract acceleration data
   * @return The acceleration data
   */
  virtual const Eigen::VectorXd& getAcceleration(Eigen::Index i) const = 0;

  /**
   * @brief Set data for a given index
   * @param i The index to set data
   * @param velocity The velocity data to assign to index
   * @param acceleration The acceleration data to assign to index
   * @param time The time from start to assign to index
   */
  virtual void
  setData(Eigen::Index i, const Eigen::VectorXd& velocity, const Eigen::VectorXd& acceleration, double time) = 0;

  /** @brief The size of the path */
  virtual Eigen::Index size() const = 0;

  /** @brief The degree of freedom for the path */
  virtual Eigen::Index dof() const = 0;

  /** @brief Check if the path is empty */
  virtual bool empty() const = 0;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_TIME_PARAMETERIZATION_TRAJECTORY_CONTAINER_H

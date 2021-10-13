/**
 * @file robot_config.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/joint_group.h>

namespace tesseract_planning
{
/**
 * @brief The RobotConfig enum
 *
 * The first letter refers to 'Flip vs No-Flip', think of the human arm as the robot this would relate to wrist flipped
 * or not flipped.
 *   - Which is indicated by flipping the sign of J5 requiring J4 and J6 to move by +/-180 degrees
 * The second letter refers to 'Up vs Down' think of the human arm as the robot this would relate to elbow up or down
 *   - Which is indicated, J3 is greater than or less than 90 degrees and tool0 position x is positive or negative
 * The third letter refers to 'Front vs Back' think of the human arm as the robot this would relate to arm in front or
 * back
 *
 */
enum class RobotConfig
{
  NUT = 0,
  FUT = 1,
  NDT = 2,
  FDT = 3,
  NDB = 4,
  FDB = 5,
  NUB = 6,
  FUB = 7
};

static const std::vector<std::string> RobotConfigString = { "NUT", "FUT", "NDT", "FDT", "NDB", "FDB", "NUB", "FUB" };

/**
 * @brief Get the configuration of a six axis industrial robot
 * @param joint_group The kinematics JointGroup.
 * @param base_link The base link to use.
 * @param tcp_frame The tip link to use.
 * @param joint_values The joint group joint values and assumes the last six are for the robot.
 * @param sign_correction Correct the sign for Joint 3 and Joint 5 based on the robot manufacturer.
 * @return Robot Config
 */
template <typename FloatType>
inline RobotConfig getRobotConfig(const tesseract_kinematics::JointGroup& joint_group,
                                  const std::string& base_link,
                                  const std::string& tcp_frame,
                                  const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                                  const Eigen::Ref<const Eigen::Vector2i>& sign_correction = Eigen::Vector2i::Ones())
{
  // Get state
  tesseract_common::TransformMap state = joint_group.calcFwdKin(joint_values.template cast<double>());

  // Get the pose at tool0
  Eigen::Isometry3d pose = state.at(base_link).inverse() * state.at(tcp_frame);

  // Get the base rotated by joint 1
  Eigen::Isometry3d prime_pose(
      Eigen::AngleAxisd(static_cast<double>(joint_values.tail(6)(0)), Eigen::Vector3d::UnitZ()));

  // Transform tool0 pose into new frame
  Eigen::Isometry3d pose_prime = prime_pose.inverse() * pose;

  // If pose_prime.x is greater than and equal to zero then it is in the forward configuration, otherwise
  // in the backward configuration.

  std::array<std::string, 3> config;

  // Wrist Flip or Not Flip
  if ((sign_correction[1] * joint_values(4)) >= 0)
    config[0] = "F";
  else
    config[0] = "N";

  // Elbow Up or Down
  if ((sign_correction[0] * joint_values(2)) < M_PI / 2)
    config[1] = "U";
  else
    config[1] = "D";

  // Robot Front or Back
  if (pose_prime.translation().x() >= 0)
    config[2] = "T";
  else
    config[2] = "B";

  if (config == std::array<std::string, 3>({ "F", "U", "T" }))
    return RobotConfig::FUT;

  if (config == std::array<std::string, 3>({ "N", "U", "T" }))
    return RobotConfig::NUT;

  if (config == std::array<std::string, 3>({ "F", "D", "T" }))
    return RobotConfig::FDT;

  if (config == std::array<std::string, 3>({ "N", "D", "T" }))
    return RobotConfig::NDT;

  if (config == std::array<std::string, 3>({ "F", "U", "B" }))
    return RobotConfig::FUB;

  if (config == std::array<std::string, 3>({ "N", "U", "B" }))
    return RobotConfig::NUB;

  if (config == std::array<std::string, 3>({ "F", "D", "B" }))
    return RobotConfig::FDB;

  return RobotConfig::NDB;
}

/**
 * @brief Get number of turns for joints that allow rotation beyond +- 180 degrees
 * @param joint values The joint values of the robot
 * @return The number of turns (as integers), in a vector
 */
template <typename FloatType>
inline Eigen::VectorXi getJointTurns(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values)
{
  Eigen::VectorXi joint_turns;
  joint_turns.resize(joint_values.rows());
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joint_turns(i) = int(joint_values(i) / M_PI);

  return joint_turns;
}

}  // namespace tesseract_planning

#ifdef SWIG
%template(getRobotConfig) tesseract_planning::getRobotConfig<double>;
%template(getJointTurns) tesseract_planning::getJointTurns<double>;
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_ROBOT_CONFIG_H

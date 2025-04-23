/**
 * @file planner_utils.h
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/constants.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_motion_planners/robot_config.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
/**
 * @brief Check if a joint is within limits
 * This will allows you to provide joint 4 and joint 6 coupling limits. This will still work if the robot is on a
 * position.
 * @param joint_values The joint values of the robot
 * @param limits The robot joint limits
 * @param coupling_limits Limits for robot joint 4 and 6 coupling.
 * @return True if within limits otherwise false
 */
template <typename FloatType>
bool isWithinJointLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                         const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits,
                         const Eigen::Vector2d& coupling_limits)
{
  if (tesseract_common::isWithinLimits<FloatType>(joint_values, limits))
  {
    if (((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) < coupling_limits(0)) ||
        ((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) > coupling_limits(1)))
      return false;
  }
  else
  {
    return false;
  }

  return true;
}

/**
 * @brief Check if the robot is in a valid state
 * @param joint_group The joint group
 * @param base_link The base link to use.
 * @param tcp_frame The tip link to use.
 * @param joint_values The joint values of the robot
 * @param limits The robot joint limits
 * @return True if within limits otherwise false
 */
template <typename FloatType>
bool isValidState(const tesseract_kinematics::JointGroup& joint_group,
                  const std::string& base_link,
                  const std::string& tcp_frame,
                  const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                  const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  if (!tesseract_common::isWithinLimits(joint_values, limits))
    return false;

  Eigen::Vector2i sign_correction = Eigen::Vector2i::Ones();
  sign_correction(0) = -1;
  RobotConfig robot_config = getRobotConfig<FloatType>(
      joint_group, base_link, tcp_frame, joint_values.tail(joint_group.numJoints()), sign_correction);

  return !(robot_config != RobotConfig::FUT && robot_config != RobotConfig::NUT);  // NOLINT
}

/**
 * @brief Gets the profile specified from the profile map
 * @param ns The namespace to search for requested profile
 * @param profile The requested profile
 * @param profile_map map that contains the profiles
 * @param default_profile Profile that is returned if the requested profile is not found in the map. Default = nullptr
 * @return The profile requested if found. Otherwise the default_profile
 */
template <typename ProfileType>
std::shared_ptr<const ProfileType> getProfile(const std::string& ns,
                                              const std::string& profile,
                                              const tesseract_common::ProfileDictionary& profile_dictionary,
                                              std::shared_ptr<const ProfileType> default_profile = nullptr)
{
  if (profile_dictionary.hasProfile(ProfileType::getStaticKey(), ns, profile))
    return std::static_pointer_cast<const ProfileType>(
        profile_dictionary.getProfile(ProfileType::getStaticKey(), ns, profile));

  std::stringstream ss;
  ss << "Profile '" << profile << "' was not found in namespace '" << ns << "' for type '"
     << boost::core::demangle(typeid(ProfileType).name()) << "'. Using default if available. Available profiles: [";
  if (profile_dictionary.hasProfileEntry(ProfileType::getStaticKey(), ns))
    for (const auto& pair : profile_dictionary.getProfileEntry(ProfileType::getStaticKey(), ns))
      ss << pair.first << ", ";
  ss << "]";

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());

  return default_profile;
}
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H

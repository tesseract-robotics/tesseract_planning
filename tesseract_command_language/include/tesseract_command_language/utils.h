/**
 * @file utils/utils.h
 * @brief General utilities and all inclusive utility header
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
#ifndef TESSERACT_COMMAND_LANGUAGE_UTILS_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_UTILS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/types.h>

namespace tesseract_planning
{
/**
 * @brief Convert instruction to a joint trajectory
 * @details This searches for both move instructions. If it contains a Cartesian waypoint it is skipped.
 * @param instruction The instruction to convert
 * @return A joint trajectory
 */
tesseract_common::JointTrajectory toJointTrajectory(const InstructionPoly& instruction);

/**
 * @brief Convert composite instruction to a joint trajectory
 * @details This searches for both move and plan instruction to support converting both input and results to planning
 * requests. If it contains a Cartesian waypoint it is skipped.
 * @param composite_instructions The composite instruction to convert
 * @return A joint trajectory
 */
tesseract_common::JointTrajectory toJointTrajectory(const CompositeInstruction& composite_instructions);

/**
 * @brief Gets joint position from waypoints that contain that information.
 *
 * Throws if waypoint does not directly contain that information
 * @param waypoint The waypoint to try and extract the joint position from
 * @return The joint values
 */
const Eigen::VectorXd& getJointPosition(const WaypointPoly& waypoint);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
const Eigen::VectorXd& getJointPosition(const T&) = delete;

/**
 * @brief Gets joint names from waypoints that contain that information.
 *
 * Throws if waypoint does not directly contain that information
 *
 * @param waypoint The waypoint to try and extract the joint position from
 * @return The joint names
 */
const std::vector<std::string>& getJointNames(const WaypointPoly& waypoint);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
const std::vector<std::string>& getJointNames(const T&) = delete;

/**
 * @brief Get the joint positions ordered by the provided joint names
 *
 * Throws if waypoint does not directly contain that information
 *
 * Also this is an expensive call so the motion planners do not leverage this and they expect the order through out
 * the program all match.
 *
 * @param joint_names The joint names defining the order desired
 * @param waypoint The waypoint to
 * @return The joint values ordered by the provided joint_names
 */
Eigen::VectorXd getJointPosition(const std::vector<std::string>& joint_names, const WaypointPoly& waypoint);

/**
 * @brief Format the waypoints joint ordered by the provided joint names
 *
 * Throws if waypoint does not directly contain that information
 *
 * Also this is an expensive call so the motion planners do not leverage this and they expect the order through out
 * the program all match.
 *
 * @param joint_names The joint names defining the order desired
 * @param waypoint The waypoint to format
 * @return True if formatting was required, otherwise false.
 */
bool formatJointPosition(const std::vector<std::string>& joint_names, WaypointPoly& waypoint);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
bool formatJointPosition(const std::vector<std::string>&, T&) = delete;

/**
 * @brief Check the waypoints joint order against the provided joint names
 *
 * Throws if waypoint does not directly contain that information
 *
 * Also this is an expensive call so the motion planners do not leverage this and they expect the order through out
 * the program all match.
 *
 * @param joint_names The joint names defining the order desired
 * @param waypoint The waypoint to check format
 * @return True if waypoint format is correct, otherwise false.
 */
bool checkJointPositionFormat(const std::vector<std::string>& joint_names, const WaypointPoly& waypoint);

/**
 * @brief Set the joint position for waypoints that contain that information
 * @param waypoint Waypoint to set
 * @param position Joint position
 * @return true if successful (if the waypoint is a supported type)
 */
bool setJointPosition(WaypointPoly& waypoint, const Eigen::Ref<const Eigen::VectorXd>& position);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
bool setJointPosition(T& waypoint, const Eigen::Ref<const Eigen::VectorXd>&) = delete;

/**
 * @brief Checks if a waypoint is
 * @param wp Waypoint to be checked. Only checks if a JointPosition or State waypoint (otherwise returns true)
 * @param limits Matrix2d of limits with first column being lower limits and second column being upper limits
 * @return True if the waypoit falls within the joint limits
 */
bool isWithinJointLimits(const WaypointPoly& wp, const Eigen::Ref<const Eigen::MatrixX2d>& limits);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
bool isWithinJointLimits(const T&, const Eigen::Ref<const Eigen::MatrixX2d>&) = delete;

/**
 * @brief Clamps a waypoint to be within joint limits
 * @param wp Waypoint to be adjusted. Does nothing if not a JointPosition or State waypoint
 * @param limits Matrix2d of limits with first column being lower limits and second column being upper limits
 * @param max_deviation. Max deviation that will be clamped
 * @return True if successful or if the waypoint doesn't contain that information.
 */
#ifndef SWIG
bool clampToJointLimits(WaypointPoly& wp,
                        const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                        double max_deviation = (std::numeric_limits<double>::max)());

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
bool clampToJointLimits(T&,
                        const Eigen::Ref<const Eigen::MatrixX2d>&,
                        double max_deviation = (std::numeric_limits<double>::max)()) = delete;

#endif

/**
 * @brief Clamps a waypoint to be within joint limits
 * @param wp Waypoint to be adjusted. Does nothing if not a JointPosition or State waypoint
 * @param limits Matrix2d of limits with first column being lower limits and second column being upper limits
 * @param max_deviation. Max deviation that will be clamped
 * @return True if successful or if the waypoint doesn't contain that information.
 */
bool clampToJointLimits(WaypointPoly& wp,
                        const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                        const Eigen::Ref<const Eigen::VectorXd>& max_deviation);

/**
 * @brief This prevent implicit cast to WaypointPoly
 * @details Since we are returning by reference if the type passed is converted
 * to a WaypointPoly then the object return is referencing a temporary value.
 */
template <class T>
bool clampToJointLimits(WaypointPoly&,
                        const Eigen::Ref<const Eigen::MatrixX2d>&,
                        const Eigen::Ref<const Eigen::VectorXd>&) = delete;

/**
 * @brief Convert a CompositeInstruction to delimited formate file by extracting all MoveInstructions
 * @param composite_instructions The CompositeInstruction to extract data from
 * @param file_path The location to save the file
 * @param separator The separator to use
 * @return true if successful
 */
bool toDelimitedFile(const CompositeInstruction& composite_instructions,
                     const std::string& file_path,
                     char separator = ',');

/**
 * @brief This loops over the instructions validates the structure
 *
 * Every plan instruction in /p composite_instruction should have a cooresponding CompositeInstruction
 *
 * @param composite_instructions
 * @param composite_seed
 * @return
 */
bool validateSeedStructure(const CompositeInstruction& composite_instructions,
                           const CompositeInstruction& composite_seed);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H

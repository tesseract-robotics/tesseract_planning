/**
 * @file move_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/types.h>
#include <tesseract_common/manipulator_info.h>

namespace tesseract_planning
{
class ProfileDictionary;
/**
 * @brief The move instruction is used when defining the results of a motion planning request
 * @details
 * This instruction contains two profiles 'profile' and 'path_profile' which are similar to industrial robots
 * termination type and Motion Options.
 *   - profile (Termination Type): is used to define a set of costs/constraints associated only with the waypoint
 * assigned to this instruction
 *   - path_profile (Motion Options): is used to define a set of costs/constraints associated only with the path taken
 * to the waypoint assigned to this instruction
 */
class MoveInstruction final : public MoveInstructionInterface
{
public:
  MoveInstruction() = default;  // Required for boost serialization do not use

  /**
   * @brief Move Instruction Constructor
   * @details This constructor automatically assigns the path profile which is only associated to the path taken to the
   * waypoint. If the motion is LINEAR/CIRCULAR it assigns the profile to the defined profile. If the motion is
   * FREESPACE/START it is left empty.
   * @param waypoint The waypoint associated with the instruction
   * @param type The type of instruction (LINEAR, FREESPACE, CIRCULAR, START)
   * @param profile The waypoint profile, which is only associated to the waypoint and not the path taken.
   * @param manipulator_info Then manipulator information
   */
  explicit MoveInstruction(WaypointPoly waypoint,
                           MoveInstructionType type,
                           std::string profile = DEFAULT_PROFILE_KEY,
                           tesseract_common::ManipulatorInfo manipulator_info = tesseract_common::ManipulatorInfo());

  /**
   * @brief Move Instruction Constructor
   * @param waypoint The waypoint associated with the instruction
   * @param type The type of instruction (LINEAR, FREESPACE, CIRCULAR, START)
   * @param profile The waypoint profile, which is only associated to the waypoint and not the path taken.
   * @param path_profile The waypoint path profile which is only associated to the path taken to the waypoint.
   * @param manipulator_info Then manipulator information
   */
  explicit MoveInstruction(WaypointPoly waypoint,
                           MoveInstructionType type,
                           std::string profile,
                           std::string path_profile,
                           tesseract_common::ManipulatorInfo manipulator_info = tesseract_common::ManipulatorInfo());

  // Instruction
  const boost::uuids::uuid& getUUID() const override final;
  void setUUID(const boost::uuids::uuid& uuid) override final;
  void regenerateUUID() override final;
  const boost::uuids::uuid& getParentUUID() const override final;
  void setParentUUID(const boost::uuids::uuid& uuid) override final;
  const std::string& getDescription() const override final;
  void setDescription(const std::string& description) override final;
  void print(const std::string& prefix = "") const override final;

  // Move Instruction
  void setMoveType(MoveInstructionType move_type) override final;
  MoveInstructionType getMoveType() const override final;

  WaypointPoly& getWaypoint() override final;
  const WaypointPoly& getWaypoint() const override final;

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info) override final;
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const override final;
  tesseract_common::ManipulatorInfo& getManipulatorInfo() override final;

  void setProfile(const std::string& profile) override final;
  const std::string& getProfile(const std::string& ns = "") const override final;

  void setPathProfile(const std::string& profile) override final;
  const std::string& getPathProfile(const std::string& ns = "") const override final;

  void setProfileOverrides(ProfileOverrides profile_overrides) override final;
  const ProfileOverrides& getProfileOverrides() const override final;

  void setPathProfileOverrides(ProfileOverrides profile_overrides) override final;
  const ProfileOverrides& getPathProfileOverrides() const override final;

  CartesianWaypointPoly createCartesianWaypoint() const override final;
  JointWaypointPoly createJointWaypoint() const override final;
  StateWaypointPoly createStateWaypoint() const override final;

  std::unique_ptr<MoveInstructionInterface> clone() const override final;

private:
  /** @brief The instructions UUID */
  boost::uuids::uuid uuid_{};

  /** @brief The parent UUID if created from createChild */
  boost::uuids::uuid parent_uuid_{};

  /** @brief The move instruction type */
  MoveInstructionType move_type_{ MoveInstructionType::FREESPACE };

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Move Instruction" };

  /** @brief The profile used for this move instruction */
  std::string profile_{ DEFAULT_PROFILE_KEY };

  /** @brief The profile used for the path to this move instruction */
  std::string path_profile_;

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  ProfileOverrides profile_overrides_;

  /** @brief Dictionary of path profiles that will override named profiles for a specific task*/
  ProfileOverrides path_profile_overrides_;

  /** @brief The assigned waypoint (Cartesian, Joint or State) */
  WaypointPoly waypoint_;

  /** @brief Contains information about the manipulator associated with this instruction*/
  tesseract_common::ManipulatorInfo manipulator_info_;

  bool equals(const MoveInstructionInterface& other) const override final;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MoveInstruction)
BOOST_CLASS_TRACKING(tesseract_planning::MoveInstruction, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H

/**
 * @file move_instruction.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <console_bridge/console.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(WaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
  if (!waypoint_.isCartesianWaypoint() && !waypoint_.isJointWaypoint() && !waypoint_.isStateWaypoint())
    throw std::runtime_error("MoveIntruction only supports waypoint types: CartesianWaypointPoly, JointWaypointPoly "
                             "and StateWaypointPoly");

  if (move_type_ == MoveInstructionType::LINEAR || move_type_ == MoveInstructionType::CIRCULAR)
    path_profile_ = profile_;
}

MoveInstruction::MoveInstruction(CartesianWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
  if (move_type_ == MoveInstructionType::LINEAR || move_type_ == MoveInstructionType::CIRCULAR)
    path_profile_ = profile_;
}

MoveInstruction::MoveInstruction(JointWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
  if (move_type_ == MoveInstructionType::LINEAR || move_type_ == MoveInstructionType::CIRCULAR)
    path_profile_ = profile_;
}

MoveInstruction::MoveInstruction(StateWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
  if (move_type_ == MoveInstructionType::LINEAR || move_type_ == MoveInstructionType::CIRCULAR)
    path_profile_ = profile_;
}

MoveInstruction::MoveInstruction(WaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 std::string path_profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , path_profile_(std::move(path_profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
  if (!waypoint_.isCartesianWaypoint() && !waypoint_.isJointWaypoint() && !waypoint_.isStateWaypoint())
    throw std::runtime_error("MoveIntruction only supports waypoint types: CartesianWaypointPoly, JointWaypointPoly "
                             "and StateWaypointPoly");
}

MoveInstruction::MoveInstruction(CartesianWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 std::string path_profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , path_profile_(std::move(path_profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
}

MoveInstruction::MoveInstruction(JointWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 std::string path_profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , path_profile_(std::move(path_profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
}

MoveInstruction::MoveInstruction(StateWaypointPoly waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 std::string path_profile,
                                 tesseract_common::ManipulatorInfo manipulator_info)
  : uuid_(boost::uuids::random_generator()())
  , move_type_(type)
  , profile_(std::move(profile))
  , path_profile_(std::move(path_profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
}

const boost::uuids::uuid& MoveInstruction::getUUID() const { return uuid_; }
void MoveInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("MoveInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void MoveInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& MoveInstruction::getParentUUID() const { return parent_uuid_; }
void MoveInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

void MoveInstruction::setMoveType(MoveInstructionType move_type) { move_type_ = move_type; }

MoveInstructionType MoveInstruction::getMoveType() const { return move_type_; }

void MoveInstruction::assignCartesianWaypoint(CartesianWaypointPoly waypoint) { waypoint_ = waypoint; }
void MoveInstruction::assignJointWaypoint(JointWaypointPoly waypoint) { waypoint_ = waypoint; }
void MoveInstruction::assignStateWaypoint(StateWaypointPoly waypoint) { waypoint_ = waypoint; }
WaypointPoly& MoveInstruction::getWaypoint() { return waypoint_; }
const WaypointPoly& MoveInstruction::getWaypoint() const { return waypoint_; }

void MoveInstruction::setManipulatorInfo(tesseract_common::ManipulatorInfo info)
{
  manipulator_info_ = std::move(info);
}
const tesseract_common::ManipulatorInfo& MoveInstruction::getManipulatorInfo() const { return manipulator_info_; }
tesseract_common::ManipulatorInfo& MoveInstruction::getManipulatorInfo() { return manipulator_info_; }

void MoveInstruction::setProfile(const std::string& profile) { profile_ = profile; }
const std::string& MoveInstruction::getProfile() const { return profile_; }

void MoveInstruction::setPathProfile(const std::string& profile) { path_profile_ = profile; }
const std::string& MoveInstruction::getPathProfile() const { return path_profile_; }

void MoveInstruction::setProfileOverrides(ProfileDictionary::ConstPtr profile_overrides)
{
  profile_overrides_ = std::move(profile_overrides);
}
ProfileDictionary::ConstPtr MoveInstruction::getProfileOverrides() const { return profile_overrides_; }

void MoveInstruction::setPathProfileOverrides(ProfileDictionary::ConstPtr profile_overrides)
{
  path_profile_overrides_ = std::move(profile_overrides);
}
ProfileDictionary::ConstPtr MoveInstruction::getPathProfileOverrides() const { return path_profile_overrides_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

void MoveInstruction::print(const std::string& prefix) const
{
  std::cout << prefix + "Move Instruction, Move Type: " << static_cast<int>(move_type_);
  if (!getWaypoint().isNull())
  {
    std::cout << ", ";
    getWaypoint().print();
  }
  std::cout << ", Description: " << getDescription() << std::endl;
}

CartesianWaypointPoly MoveInstruction::createCartesianWaypoint() { return CartesianWaypoint(); }
JointWaypointPoly MoveInstruction::createJointWaypoint() { return JointWaypoint(); }
StateWaypointPoly MoveInstruction::createStateWaypoint() { return StateWaypoint(); }

bool MoveInstruction::operator==(const MoveInstruction& rhs) const
{
  bool equal = true;
  equal &= (static_cast<int>(move_type_) == static_cast<int>(rhs.move_type_));
  equal &= (waypoint_ == rhs.waypoint_);
  equal &= (manipulator_info_ == rhs.manipulator_info_);
  equal &= (profile_ == rhs.profile_);            // NO LINT
  equal &= (path_profile_ == rhs.path_profile_);  // NO LINT
  /** @todo Add profiles overrides when serialization is supported for profiles */
  return equal;
}
// LCOV_EXCL_START
bool MoveInstruction::operator!=(const MoveInstruction& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void MoveInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("move_type", move_type_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("profile", profile_);
  ar& boost::serialization::make_nvp("path_profile", path_profile_);
  ar& boost::serialization::make_nvp("waypoint", waypoint_);
  ar& boost::serialization::make_nvp("manipulator_info", manipulator_info_);
  /** @todo Add profiles overrides when serialization is supported for profiles */
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MoveInstruction)
TESSERACT_MOVE_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::MoveInstruction)

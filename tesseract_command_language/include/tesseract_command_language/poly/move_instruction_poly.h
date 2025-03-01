/**
 * @file move_instruction_poly.h
 * @brief The move instruction interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <cstdint>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/types.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class WaypointPoly;
class CartesianWaypointPoly;
class JointWaypointPoly;
class StateWaypointPoly;

enum class MoveInstructionType : std::uint8_t
{
  LINEAR = 0,
  FREESPACE = 1,
  CIRCULAR = 2,
};

class MoveInstructionInterface
{
public:
  virtual ~MoveInstructionInterface() = default;

  //////////////
  // Instruction
  //////////////

  /**
   * @brief Get the UUID
   * @return The UUID
   */
  virtual const boost::uuids::uuid& getUUID() const = 0;
  /**
   * @brief Set the UUID
   * @param uuid The UUID
   */
  virtual void setUUID(const boost::uuids::uuid& uuid) = 0;
  /**
   * @brief Regenerate the UUID
   */
  virtual void regenerateUUID() = 0;

  /**
   * @brief Get the parent UUID
   * @return The parent UUID
   */
  virtual const boost::uuids::uuid& getParentUUID() const = 0;
  /**
   * @brief Set the parent UUID
   * @param uuid The parent UUID
   */
  virtual void setParentUUID(const boost::uuids::uuid& uuid) = 0;

  /**
   * @brief Get the description
   * @return The description
   */
  virtual const std::string& getDescription() const = 0;
  /**
   * @brief Set the description
   * @param description The description
   */
  virtual void setDescription(const std::string& description) = 0;

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  virtual void print(const std::string& prefix) const = 0;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  virtual std::unique_ptr<MoveInstructionInterface> clone() const = 0;

  ///////////////////
  // Move Instruction
  ///////////////////

  /**
   * @brief Get the waypoint
   * @return The waypoint
   */
  virtual WaypointPoly& getWaypoint() = 0;
  virtual const WaypointPoly& getWaypoint() const = 0;

  /**
   * @brief Set the manipulator information
   * @param info The manipulator information
   */
  virtual void setManipulatorInfo(tesseract_common::ManipulatorInfo info) = 0;
  /**
   * @brief Get the manipulator information
   * @return The manipulator information
   */
  virtual const tesseract_common::ManipulatorInfo& getManipulatorInfo() const = 0;
  virtual tesseract_common::ManipulatorInfo& getManipulatorInfo() = 0;

  /**
   * @brief Set the waypoint profile
   * @param profile The waypoint profile
   */
  virtual void setProfile(const std::string& profile) = 0;
  /**
   * @brief Get the waypoint profile
   * @param ns The namepace to lookup profile
   * @return The profile associated with the namespace
   */
  virtual const std::string& getProfile(const std::string& ns = "") const = 0;

  /**
   * @brief Set the waypoint path profile
   * @param profile The waypoint path profile
   */
  virtual void setPathProfile(const std::string& profile) = 0;
  /**
   * @brief Get the waypoint path profile
   * @param ns The namepace to lookup profile
   * @return The path profile associated with the namespace
   */
  virtual const std::string& getPathProfile(const std::string& ns = "") const = 0;

  /**
   * @brief Set the profile overrides
   * @param profile_overrides The profile overrides
   */
  virtual void setProfileOverrides(ProfileOverrides profile_overrides) = 0;
  /**
   * @brief Get the profile overrides
   * @return The profile overrides
   */
  virtual const ProfileOverrides& getProfileOverrides() const = 0;

  /**
   * @brief Set the path profile overrides
   * @param profile_overrides The path profile overrides
   */
  virtual void setPathProfileOverrides(ProfileOverrides profile_overrides) = 0;
  /**
   * @brief Get the path profile overrides
   * @return The path profile overrides
   */
  virtual const ProfileOverrides& getPathProfileOverrides() const = 0;

  /**
   * @brief Set the move type
   * @param move_type The move type
   */
  virtual void setMoveType(MoveInstructionType move_type) = 0;
  /**
   * @brief Get the move type
   * @return The move type
   */
  virtual MoveInstructionType getMoveType() const = 0;

  /**
   * @brief Create cartesian waypoint poly
   * @return A cartesian waypoint poly
   */
  virtual CartesianWaypointPoly createCartesianWaypoint() const = 0;
  /**
   * @brief Create joint waypoint poly
   * @return A joint waypoint poly
   */
  virtual JointWaypointPoly createJointWaypoint() const = 0;
  /**
   * @brief Create state waypoint poly
   * @return A state waypoint poly
   */
  virtual StateWaypointPoly createStateWaypoint() const = 0;

  // Operators
  bool operator==(const MoveInstructionInterface& rhs) const;
  bool operator!=(const MoveInstructionInterface& rhs) const;

protected:
  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  virtual bool equals(const MoveInstructionInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class MoveInstructionPoly final : public InstructionInterface
{
public:
  MoveInstructionPoly() = default;  // Default constructor
  MoveInstructionPoly(const MoveInstructionPoly& other);
  MoveInstructionPoly& operator=(const MoveInstructionPoly& other);
  MoveInstructionPoly(MoveInstructionPoly&& other) noexcept = default;
  MoveInstructionPoly& operator=(MoveInstructionPoly&& other) noexcept = default;
  MoveInstructionPoly(const MoveInstructionInterface& impl);

  //////////////
  // Instruction
  //////////////

  /**
   * @brief Get the UUID
   * @return The UUID
   */
  const boost::uuids::uuid& getUUID() const final override;
  /**
   * @brief Set the UUID
   * @param uuid The UUID
   */
  void setUUID(const boost::uuids::uuid& uuid) final override;
  /**
   * @brief Regenerate the UUID
   */
  void regenerateUUID() final override;

  /**
   * @brief Get the parent UUID
   * @return The parent UUID
   */
  const boost::uuids::uuid& getParentUUID() const final override;
  /**
   * @brief Set the parent UUID
   * @param uuid The parent UUID
   */
  void setParentUUID(const boost::uuids::uuid& uuid) final override;

  /**
   * @brief Get the description
   * @return The description
   */
  const std::string& getDescription() const final override;
  /**
   * @brief Set the description
   * @param description The description
   */
  void setDescription(const std::string& description) final override;

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  void print(const std::string& prefix = "") const final override;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  std::unique_ptr<InstructionInterface> clone() const override final;

  ///////////////////
  // Move Instruction
  ///////////////////

  /**
   * @brief Get the waypoint
   * @return The waypoint
   */
  WaypointPoly& getWaypoint();
  const WaypointPoly& getWaypoint() const;

  /**
   * @brief Set the manipulator information
   * @param info The manipulator information
   */
  void setManipulatorInfo(tesseract_common::ManipulatorInfo info);
  /**
   * @brief Get the manipulator information
   * @return The manipulator information
   */
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const;
  tesseract_common::ManipulatorInfo& getManipulatorInfo();

  /**
   * @brief Set the waypoint profile
   * @param profile The waypoint profile
   */
  void setProfile(const std::string& profile);
  /**
   * @brief Get the waypoint profile
   * @param ns The namepace to lookup profile
   * @return The profile associated with the namespace
   */
  const std::string& getProfile(const std::string& ns = "") const;

  /**
   * @brief Set the waypoint path profile
   * @param profile The waypoint path profile
   */
  void setPathProfile(const std::string& profile);
  /**
   * @brief Get the waypoint path profile
   * @param ns The namepace to lookup profile
   * @return The path profile associated with the namespace
   */
  const std::string& getPathProfile(const std::string& ns = "") const;

  /**
   * @brief Set the profile overrides
   * @param profile_overrides The profile overrides
   */
  void setProfileOverrides(ProfileOverrides profile_overrides);
  /**
   * @brief Get the profile overrides
   * @return The profile overrides
   */
  const ProfileOverrides& getProfileOverrides() const;

  /**
   * @brief Set the path profile overrides
   * @param profile_overrides The path profile overrides
   */
  void setPathProfileOverrides(ProfileOverrides profile_overrides);
  /**
   * @brief Get the path profile overrides
   * @return The path profile overrides
   */
  const ProfileOverrides& getPathProfileOverrides() const;

  /**
   * @brief Set the move type
   * @param move_type The move type
   */
  void setMoveType(MoveInstructionType move_type);
  /**
   * @brief Get the move type
   * @return The move type
   */
  MoveInstructionType getMoveType() const;

  /**
   * @brief Create cartesian waypoint poly
   * @return A cartesian waypoint poly
   */
  CartesianWaypointPoly createCartesianWaypoint() const;
  /**
   * @brief Create joint waypoint poly
   * @return A joint waypoint poly
   */
  JointWaypointPoly createJointWaypoint() const;
  /**
   * @brief Create state waypoint poly
   * @return A state waypoint poly
   */
  StateWaypointPoly createStateWaypoint() const;

  ///////////////
  // Poly methods
  ///////////////

  /**
   * @brief Get the stored derived type
   * @return The derived type index
   */
  std::type_index getType() const;

  /**
   * @brief Check if the poly type is null
   * @return True if null, otherwise false
   */
  bool isNull() const;

  /**
   * @brief Get the move instruction being stored
   * @return The move instruction
   * @throws If null
   */
  MoveInstructionInterface& getMoveInstruction();
  const MoveInstructionInterface& getMoveInstruction() const;

  /**
   * @brief Create child move instruction
   * @return A child move instruction
   */
  MoveInstructionPoly createChild() const;

  /**
   * @brief Check if move type is linear
   * @return True if move type is linear, otherwise false
   */
  bool isLinear() const;

  /**
   * @brief Check if move type is freespace
   * @return True if move type is freespace, otherwise false
   */
  bool isFreespace() const;

  /**
   * @brief Check if move type is circular
   * @return True if move type is circular, otherwise false
   */
  bool isCircular() const;

  /**
   * @brief Check if move instruction is a child
   * @return True if move instruction is a child, otherwise false
   */
  bool isChild() const;

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("MoveInstructionPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("MoveInstructionPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const MoveInstructionPoly& rhs) const;
  bool operator!=(const MoveInstructionPoly& rhs) const;

private:
  std::unique_ptr<MoveInstructionInterface> impl_;

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  bool equals(const InstructionInterface& other) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MoveInstructionInterface)
BOOST_CLASS_TRACKING(tesseract_planning::MoveInstructionInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MoveInstructionPoly)
BOOST_CLASS_TRACKING(tesseract_planning::MoveInstructionPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_H

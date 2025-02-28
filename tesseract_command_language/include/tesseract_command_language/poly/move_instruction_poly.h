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

  virtual const boost::uuids::uuid& getUUID() const = 0;
  virtual void setUUID(const boost::uuids::uuid& uuid) = 0;
  virtual void regenerateUUID() = 0;

  virtual const boost::uuids::uuid& getParentUUID() const = 0;
  virtual void setParentUUID(const boost::uuids::uuid& uuid) = 0;

  virtual const std::string& getDescription() const = 0;
  virtual void setDescription(const std::string& description) = 0;

  virtual void print(const std::string& prefix) const = 0;

  virtual std::unique_ptr<MoveInstructionInterface> clone() const = 0;

  ///////////////////
  // Move Instruction
  ///////////////////

  virtual WaypointPoly& getWaypoint() = 0;
  virtual const WaypointPoly& getWaypoint() const = 0;

  virtual void setManipulatorInfo(tesseract_common::ManipulatorInfo info) = 0;
  virtual const tesseract_common::ManipulatorInfo& getManipulatorInfo() const = 0;
  virtual tesseract_common::ManipulatorInfo& getManipulatorInfo() = 0;

  virtual void setProfile(const std::string& profile) = 0;
  virtual const std::string& getProfile(const std::string& ns = "") const = 0;

  virtual void setPathProfile(const std::string& profile) = 0;
  virtual const std::string& getPathProfile(const std::string& ns = "") const = 0;

  virtual void setProfileOverrides(ProfileOverrides profile_overrides) = 0;
  virtual const ProfileOverrides& getProfileOverrides() const = 0;

  virtual void setPathProfileOverrides(ProfileOverrides profile_overrides) = 0;
  virtual const ProfileOverrides& getPathProfileOverrides() const = 0;

  virtual void setMoveType(MoveInstructionType move_type) = 0;
  virtual MoveInstructionType getMoveType() const = 0;

  virtual CartesianWaypointPoly createCartesianWaypoint() const = 0;
  virtual JointWaypointPoly createJointWaypoint() const = 0;
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

  const boost::uuids::uuid& getUUID() const final override;
  void setUUID(const boost::uuids::uuid& uuid) final override;
  void regenerateUUID() final override;

  const boost::uuids::uuid& getParentUUID() const final override;
  void setParentUUID(const boost::uuids::uuid& uuid) final override;

  const std::string& getDescription() const final override;
  void setDescription(const std::string& description) final override;

  void print(const std::string& prefix = "") const final override;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  std::unique_ptr<InstructionInterface> clone() const override final;

  ///////////////////
  // Move Instruction
  ///////////////////

  WaypointPoly& getWaypoint();
  const WaypointPoly& getWaypoint() const;

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info);
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const;
  tesseract_common::ManipulatorInfo& getManipulatorInfo();

  void setProfile(const std::string& profile);
  const std::string& getProfile(const std::string& ns = "") const;

  void setPathProfile(const std::string& profile);
  const std::string& getPathProfile(const std::string& ns = "") const;

  void setProfileOverrides(ProfileOverrides profile_overrides);
  const ProfileOverrides& getProfileOverrides() const;

  void setPathProfileOverrides(ProfileOverrides profile_overrides);
  const ProfileOverrides& getPathProfileOverrides() const;

  void setMoveType(MoveInstructionType move_type);
  MoveInstructionType getMoveType() const;

  CartesianWaypointPoly createCartesianWaypoint() const;
  JointWaypointPoly createJointWaypoint() const;
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

  MoveInstructionPoly createChild() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

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

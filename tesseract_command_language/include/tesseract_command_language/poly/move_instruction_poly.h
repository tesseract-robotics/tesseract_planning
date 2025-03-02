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
#include <boost/serialization/base_object.hpp>
#include <boost/concept_check.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/types.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/type_erasure.h>
#include <tesseract_common/fwd.h>

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_MOVE_INSTRUCTION_EXPORT_KEY(N, C)                                                                    \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##InstanceBase =                                                                                              \
      tesseract_common::TypeErasureInstance<C, tesseract_planning::detail_move_instruction::MoveInstructionInterface>; \
  using C##Instance = tesseract_planning::detail_move_instruction::MoveInstructionInstance<C>;                         \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceBase)                                                                           \
  BOOST_CLASS_EXPORT_KEY(N::C##Instance)                                                                               \
  BOOST_CLASS_TRACKING(N::C##InstanceBase, boost::serialization::track_never)                                          \
  BOOST_CLASS_TRACKING(N::C##Instance, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_MOVE_INSTRUCTION_EXPORT_IMPLEMENT(inst)                                                              \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceBase)                                                                     \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##Instance)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_MOVE_INSTRUCTION_EXPORT(N, C)                                                                        \
  TESSERACT_MOVE_INSTRUCTION_EXPORT_KEY(N, C)                                                                          \
  TESSERACT_MOVE_INSTRUCTION_EXPORT_IMPLEMENT(N::C)

namespace tesseract_planning
{
struct MoveInstructionPoly;
class ProfileDictionary;

enum class MoveInstructionType : std::uint8_t
{
  LINEAR = 0,
  FREESPACE = 1,
  CIRCULAR = 2,
};
}  // namespace tesseract_planning

namespace tesseract_planning::detail_move_instruction
{
template <typename T>
struct MoveInstructionConcept  // NOLINT
  : boost::Assignable<T>,
    boost::CopyConstructible<T>,
    boost::EqualityComparable<T>
{
  BOOST_CONCEPT_USAGE(MoveInstructionConcept)
  {
    T cp(c);
    T assign = c;
    bool eq = (c == cp);
    bool neq = (c != cp);
    UNUSED(assign);
    UNUSED(eq);
    UNUSED(neq);

    const auto& uuid = c.getUUID();

    c.setUUID(uuid);
    c.regenerateUUID();
    c.setParentUUID(uuid);

    const auto& parent_uuid = c.getParentUUID();
    UNUSED(parent_uuid);

    tesseract_common::ManipulatorInfo info;
    c.setManipulatorInfo(info);

    tesseract_common::ManipulatorInfo& info_ref = c.getManipulatorInfo();
    UNUSED(info_ref);

    const tesseract_common::ManipulatorInfo& info_const_ref = c.getManipulatorInfo();
    UNUSED(info_const_ref);

    c.setProfile("profile");
    const std::string& profile_const = c.getProfile();
    UNUSED(profile_const);

    c.setPathProfile("path_profile");
    const std::string& path_profile_const = c.getPathProfile();
    UNUSED(path_profile_const);

    c.setProfileOverrides(ProfileOverrides{});
    auto profile_overrides = c.getProfileOverrides();
    UNUSED(profile_overrides);

    c.setPathProfileOverrides(ProfileOverrides{});
    auto path_profile_overrides = c.getPathProfileOverrides();
    UNUSED(path_profile_overrides);

    c.setMoveType(MoveInstructionType::FREESPACE);
    MoveInstructionType type = c.getMoveType();
    UNUSED(type);

    const std::string& desc = c.getDescription();
    UNUSED(desc);

    auto cwp = c.createCartesianWaypoint();
    UNUSED(cwp);

    auto jwp = c.createJointWaypoint();
    UNUSED(jwp);

    auto swp = c.createStateWaypoint();
    UNUSED(swp);

    c.setDescription("test");
    c.print();
    c.print("prefix_");
  }

private:
  T c;
};

struct MoveInstructionInterface : tesseract_common::TypeErasureInterface
{
  virtual const boost::uuids::uuid& getUUID() const = 0;
  virtual void setUUID(const boost::uuids::uuid& uuid) = 0;
  virtual void regenerateUUID() = 0;

  virtual const boost::uuids::uuid& getParentUUID() const = 0;
  virtual void setParentUUID(const boost::uuids::uuid& uuid) = 0;

  virtual void assignCartesianWaypoint(CartesianWaypointPoly waypoint) = 0;
  virtual void assignJointWaypoint(JointWaypointPoly waypoint) = 0;
  virtual void assignStateWaypoint(StateWaypointPoly waypoint) = 0;
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

  virtual const std::string& getDescription() const = 0;
  virtual void setDescription(const std::string& description) = 0;

  virtual void print(const std::string& prefix) const = 0;

  virtual CartesianWaypointPoly createCartesianWaypoint() const = 0;
  virtual JointWaypointPoly createJointWaypoint() const = 0;
  virtual StateWaypointPoly createStateWaypoint() const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

template <typename T>
struct MoveInstructionInstance : tesseract_common::TypeErasureInstance<T, MoveInstructionInterface>  // NOLINT
{
  using BaseType = tesseract_common::TypeErasureInstance<T, MoveInstructionInterface>;
  MoveInstructionInstance() = default;
  MoveInstructionInstance(const T& x) : BaseType(x) {}
  MoveInstructionInstance(MoveInstructionInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((MoveInstructionConcept<T>));

  const boost::uuids::uuid& getUUID() const final { return this->get().getUUID(); }
  void setUUID(const boost::uuids::uuid& uuid) final { this->get().setUUID(uuid); }
  void regenerateUUID() final { this->get().regenerateUUID(); }

  const boost::uuids::uuid& getParentUUID() const final { return this->get().getParentUUID(); }
  void setParentUUID(const boost::uuids::uuid& uuid) final { this->get().setParentUUID(uuid); }

  void assignCartesianWaypoint(CartesianWaypointPoly waypoint) final
  {
    this->get().assignCartesianWaypoint(std::move(waypoint));
  }
  void assignJointWaypoint(JointWaypointPoly waypoint) final { this->get().assignJointWaypoint(std::move(waypoint)); }
  void assignStateWaypoint(StateWaypointPoly waypoint) final { this->get().assignStateWaypoint(std::move(waypoint)); }
  WaypointPoly& getWaypoint() final { return this->get().getWaypoint(); }
  const WaypointPoly& getWaypoint() const final { return this->get().getWaypoint(); }

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info) final
  {
    this->get().setManipulatorInfo(std::move(info));
  }
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const final { return this->get().getManipulatorInfo(); }
  tesseract_common::ManipulatorInfo& getManipulatorInfo() final { return this->get().getManipulatorInfo(); }

  void setProfile(const std::string& profile) final { this->get().setProfile(profile); }
  const std::string& getProfile(const std::string& ns = "") const final { return this->get().getProfile(ns); }

  void setPathProfile(const std::string& profile) final { this->get().setPathProfile(profile); }
  const std::string& getPathProfile(const std::string& ns = "") const final { return this->get().getPathProfile(ns); }

  void setProfileOverrides(ProfileOverrides profile_overrides) final
  {
    this->get().setProfileOverrides(std::move(profile_overrides));
  }
  const ProfileOverrides& getProfileOverrides() const final { return this->get().getProfileOverrides(); }

  void setPathProfileOverrides(ProfileOverrides profile_overrides) final
  {
    this->get().setPathProfileOverrides(std::move(profile_overrides));
  }
  const ProfileOverrides& getPathProfileOverrides() const final { return this->get().getPathProfileOverrides(); }

  void setMoveType(MoveInstructionType move_type) final { this->get().setMoveType(move_type); }
  MoveInstructionType getMoveType() const final { return this->get().getMoveType(); }

  const std::string& getDescription() const final { return this->get().getDescription(); }
  void setDescription(const std::string& description) final { this->get().setDescription(description); }

  void print(const std::string& prefix) const final { this->get().print(prefix); }

  CartesianWaypointPoly createCartesianWaypoint() const final { return this->get().createCartesianWaypoint(); }
  JointWaypointPoly createJointWaypoint() const final { return this->get().createJointWaypoint(); }
  StateWaypointPoly createStateWaypoint() const final { return this->get().createStateWaypoint(); }

  std::unique_ptr<tesseract_common::TypeErasureInterface> clone() const final
  {
    return std::make_unique<MoveInstructionInstance<T>>(this->get());
  }

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<BaseType>(*this));
  }
};
}  // namespace tesseract_planning::detail_move_instruction

namespace tesseract_planning
{
using MoveInstructionPolyBase = tesseract_common::TypeErasureBase<detail_move_instruction::MoveInstructionInterface,
                                                                  detail_move_instruction::MoveInstructionInstance>;
struct MoveInstructionPoly : MoveInstructionPolyBase
{
  using MoveInstructionPolyBase::MoveInstructionPolyBase;

  const boost::uuids::uuid& getUUID() const;
  void setUUID(const boost::uuids::uuid& uuid);
  void regenerateUUID();

  const boost::uuids::uuid& getParentUUID() const;
  void setParentUUID(const boost::uuids::uuid& uuid);

  void assignCartesianWaypoint(CartesianWaypointPoly waypoint);
  void assignJointWaypoint(JointWaypointPoly waypoint);
  void assignStateWaypoint(StateWaypointPoly waypoint);
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

  const std::string& getDescription() const;
  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;

  CartesianWaypointPoly createCartesianWaypoint() const;
  JointWaypointPoly createJointWaypoint() const;
  StateWaypointPoly createStateWaypoint() const;

  // MoveInstructionPoly methods

  MoveInstructionPoly createChild() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

  bool isChild() const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::detail_move_instruction::MoveInstructionInterface)
BOOST_CLASS_TRACKING(tesseract_planning::detail_move_instruction::MoveInstructionInterface,
                     boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MoveInstructionPolyBase)
BOOST_CLASS_TRACKING(tesseract_planning::MoveInstructionPolyBase, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MoveInstructionPoly)
BOOST_CLASS_TRACKING(tesseract_planning::MoveInstructionPoly, boost::serialization::track_never)

TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning, MoveInstructionPoly)

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_H

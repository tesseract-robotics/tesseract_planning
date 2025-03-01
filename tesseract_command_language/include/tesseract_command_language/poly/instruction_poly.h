/**
 * @file instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <boost/stacktrace.hpp>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <string>
#include <memory>
#include <typeindex>
#include <tesseract_common/fwd.h>
#include <tesseract_common/serialization.h>

namespace boost::uuids
{
struct uuid;
}

namespace tesseract_planning
{
/**
 * @brief The InstructionInterface class
 */
class InstructionInterface
{
public:
  virtual ~InstructionInterface() = default;

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
  virtual std::unique_ptr<InstructionInterface> clone() const = 0;

  // Operators
  bool operator==(const InstructionInterface& rhs) const;
  bool operator!=(const InstructionInterface& rhs) const;

protected:
  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  virtual bool equals(const InstructionInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class InstructionPoly
{
public:
  InstructionPoly() = default;  // Default constructor
  InstructionPoly(const InstructionPoly& other);
  InstructionPoly& operator=(const InstructionPoly& other);
  InstructionPoly(InstructionPoly&& other) noexcept = default;
  InstructionPoly& operator=(InstructionPoly&& other) noexcept = default;
  InstructionPoly(const InstructionInterface& impl);

  /**
   * @brief Get the UUID
   * @return The UUID
   */
  const boost::uuids::uuid& getUUID() const;
  /**
   * @brief Set the UUID
   * @param uuid The UUID
   */
  void setUUID(const boost::uuids::uuid& uuid);
  /**
   * @brief Regenerate the UUID
   */
  void regenerateUUID();

  /**
   * @brief Get the parent UUID
   * @return The parent UUID
   */
  const boost::uuids::uuid& getParentUUID() const;
  /**
   * @brief Set the parent UUID
   * @param uuid The parent UUID
   */
  void setParentUUID(const boost::uuids::uuid& uuid);

  /**
   * @brief Get the description
   * @return The description
   */
  const std::string& getDescription() const;
  /**
   * @brief Set the description
   * @param description The description
   */
  void setDescription(const std::string& description);

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  void print(const std::string& prefix = "") const;

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
   * @brief Get the instruction being stored
   * @return The instruction
   * @throws If null
   */
  InstructionInterface& getInstruction();
  const InstructionInterface& getInstruction() const;

  /**
   * @brief Check if object being stored is of type CompositeInstruction
   * @return True if of type CompositeInstruction, otherwise false
   */
  bool isCompositeInstruction() const;

  /**
   * @brief Check if object being stored is of type MoveInstructionPoly
   * @return True if of type MoveInstructionPoly, otherwise false
   */
  bool isMoveInstruction() const;

  // Type Casting
  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("InstructionPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("InstructionPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const InstructionPoly& rhs) const;
  bool operator!=(const InstructionPoly& rhs) const;

private:
  std::unique_ptr<InstructionInterface> impl_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::InstructionInterface)
BOOST_CLASS_TRACKING(tesseract_planning::InstructionInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::InstructionPoly)
BOOST_CLASS_TRACKING(tesseract_planning::InstructionPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H

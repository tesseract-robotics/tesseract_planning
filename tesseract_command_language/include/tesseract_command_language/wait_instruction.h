/**
 * @file wait_instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date November 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_WAIT_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_WAIT_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <cstdint>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>

namespace tesseract_planning
{
enum class WaitInstructionType : std::uint8_t
{
  TIME = 0,
  DIGITAL_INPUT_HIGH = 1,
  DIGITAL_INPUT_LOW = 2,
  DIGITAL_OUTPUT_HIGH = 3,
  DIGITAL_OUTPUT_LOW = 4
};

/**
 * @brief This is a wait instruction similar to wait instruction on industrial controllers.
 * @details The instruction has several modes of operation.
 *
 *   - TIME                : This will wait for a specified number of seconds and then continue
 *   - DIGITAL_INPUT_HIGH  : This will wait for a digital input to go high(1) then continue
 *   - DIGITAL_INPUT_LOW   : This will wait for a digital input to go low(0) then continue
 *   - DIGITAL_OUTPUT_HIGH : This will wait for a digital output to go high(1) then continue
 *   - DIGITAL_OUTPUT_LOW  : This will wait for a digital output to go low(0) then continue
 */
class WaitInstruction final : public InstructionInterface
{
public:
  WaitInstruction() = default;  // Required for boost serialization do not use
  WaitInstruction(double time);
  WaitInstruction(WaitInstructionType type, int io);

  // Instruction

  /**
   * @brief Get the UUID
   * @return The UUID
   */
  const boost::uuids::uuid& getUUID() const override final;
  /**
   * @brief Set the UUID
   * @param uuid The UUID
   */
  void setUUID(const boost::uuids::uuid& uuid) override final;
  /**
   * @brief Regenerate the UUID
   */
  void regenerateUUID() override final;

  /**
   * @brief Get the parent UUID
   * @return The parent UUID
   */
  const boost::uuids::uuid& getParentUUID() const override final;
  /**
   * @brief Set the parent UUID
   * @param uuid The parent UUID
   */
  void setParentUUID(const boost::uuids::uuid& uuid) override final;

  /**
   * @brief Get the description
   * @return The description
   */
  const std::string& getDescription() const override final;
  /**
   * @brief Set the description
   * @param description The description
   */
  void setDescription(const std::string& description) override final;

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  void print(const std::string& prefix = "") const override final;  // NOLINT

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  std::unique_ptr<InstructionInterface> clone() const override final;

  // SetAnalogInstruction

  /**
   * @brief Get the wait type
   * @return The wait type
   */
  WaitInstructionType getWaitType() const;

  /**
   * @brief Set the wait type
   * @param type The wait type
   */
  void setWaitType(WaitInstructionType type);

  /**
   * @brief Get wait time in second
   * @return The wait time in second
   */
  double getWaitTime() const;
  /**
   * @brief Set wait time in second
   * @param time The wait time in second
   */
  void setWaitTime(double time);

  /**
   * @brief Get the wait IO
   * @return The wait IO
   */
  int getWaitIO() const;

  /**
   * @brief Set the wait IO
   * @param io The wait IO
   */
  void setWaitIO(int io);

private:
  /** @brief The instructions UUID */
  boost::uuids::uuid uuid_{};
  /** @brief The parent UUID if created from createChild */
  boost::uuids::uuid parent_uuid_{};
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Wait Instruction" };
  WaitInstructionType wait_type_{ WaitInstructionType::TIME };
  double wait_time_{ 0 };
  int wait_io_{ -1 };

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  bool equals(const InstructionInterface& other) const override final;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::WaitInstruction)
BOOST_CLASS_TRACKING(tesseract_planning::WaitInstruction, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_WAIT_INSTRUCTION_H

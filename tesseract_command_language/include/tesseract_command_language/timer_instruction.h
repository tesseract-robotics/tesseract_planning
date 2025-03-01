/**
 * @file timer_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <cstdint>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>

namespace tesseract_planning
{
enum class TimerInstructionType : std::uint8_t
{
  DIGITAL_OUTPUT_HIGH = 0,
  DIGITAL_OUTPUT_LOW = 1
};

/**
 * @brief This instruction indicates that a timer should be started and when the time expires it either sets a digital
 * output high(1) or low(0).
 *
 *   - DIGITAL_OUTPUT_HIGH : The digital output will be set to high(1) when the timer expires
 *   - DIGITAL_OUTPUT_LOW  : The digital output will be set to low(0) when the timer expires
 */
class TimerInstruction final : public InstructionInterface
{
public:
  TimerInstruction() = default;  // Required for boost serialization do not use
  TimerInstruction(TimerInstructionType type, double time, int io);

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

  // TimerInstruction

  /**
   * @brief Get the timer type
   * @return The timer type
   */
  TimerInstructionType getTimerType() const;

  /**
   * @brief Set the timer type
   * @param type The timer type
   */
  void setTimerType(TimerInstructionType type);

  /**
   * @brief Get timer time in second
   * @return The timer time in second
   */
  double getTimerTime() const;
  /**
   * @brief Set timer time in second
   * @param time The timer time in second
   */
  void setTimerTime(double time);

  /**
   * @brief Get the timer IO
   * @return The timer IO
   */
  int getTimerIO() const;

  /**
   * @brief Set the timer IO
   * @param io The timer IO
   */
  void setTimerIO(int io);

private:
  /** @brief The instructions UUID */
  boost::uuids::uuid uuid_{};
  /** @brief The parent UUID if created from createChild */
  boost::uuids::uuid parent_uuid_{};
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Timer Instruction" };
  TimerInstructionType timer_type_{ TimerInstructionType::DIGITAL_OUTPUT_LOW };
  double timer_time_{ 0 };
  int timer_io_{ -1 };

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

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TimerInstruction)
BOOST_CLASS_TRACKING(tesseract_planning::TimerInstruction, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H

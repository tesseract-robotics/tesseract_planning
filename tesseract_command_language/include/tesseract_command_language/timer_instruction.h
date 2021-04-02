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
#include <iostream>
#include <string>
#include <tinyxml2.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

namespace tesseract_planning
{
enum class TimerInstructionType : int
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
class TimerInstruction
{
public:
  TimerInstruction() = default;  // Required for boost serialization do not use
  TimerInstruction(TimerInstructionType type, double time, int io) : timer_type_(type), timer_time_(time), timer_io_(io)
  {
  }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(const std::string& prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Timer Instruction, Timer Type: " << static_cast<int>(timer_type_)
              << ", Time: " << timer_time_ << ", IO: " << timer_io_;
    std::cout << ", Description: " << getDescription() << std::endl;
  }

  /**
   * @brief Get the timer type
   * @return The timer type
   */
  TimerInstructionType getTimerType() const { return timer_type_; }

  /**
   * @brief Set the timer type
   * @param type The timer type
   */
  void setTimerType(TimerInstructionType type) { timer_type_ = type; }

  /**
   * @brief Get timer time in second
   * @return The timer time in second
   */
  double getTimerTime() const { return timer_time_; }
  /**
   * @brief Set timer time in second
   * @param time The timer time in second
   */
  void setTimerTime(double time) { timer_time_ = time; }

  /**
   * @brief Get the timer IO
   * @return The timer IO
   */
  int getTimerIO() const { return timer_io_; }

  /**
   * @brief Set the timer IO
   * @param io The timer IO
   */
  void setTimerIO(int io) { timer_io_ = io; }

  /**
   * @brief Equal operator. Does not compare descriptions
   * @param rhs TimerInstruction
   * @return True if equal, otherwise false
   */
  bool operator==(const TimerInstruction& rhs) const
  {
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    bool equal = true;
    equal &= tesseract_common::almostEqualRelativeAndAbs(timer_time_, rhs.timer_time_, max_diff);
    equal &= (timer_type_ == rhs.timer_type_);
    equal &= (timer_io_ == rhs.timer_io_);

    return equal;
  }

  /**
   * @brief Not equal operator. Does not compare descriptions
   * @param rhs TimerInstruction
   * @return True if not equal, otherwise false
   */
  bool operator!=(const TimerInstruction& rhs) const { return !operator==(rhs); }

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Timer Instruction" };
  TimerInstructionType timer_type_;
  double timer_time_{ 0 };
  int timer_io_{ -1 };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& boost::serialization::make_nvp("description", description_);
    ar& boost::serialization::make_nvp("timer_type", timer_type_);
    ar& boost::serialization::make_nvp("timer_time", timer_time_);
    ar& boost::serialization::make_nvp("timer_io", timer_io_);
  }
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(TimerInstruction)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H

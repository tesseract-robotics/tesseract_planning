/**
 * @file set_analog_instruction.h
 * @brief Set Analog Instruction
 *
 * @author Levi Armstrong
 * @date March 23, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>

namespace tesseract_planning
{
class SetAnalogInstruction
{
public:
  SetAnalogInstruction() = default;  // Required for boost serialization do not use
  SetAnalogInstruction(std::string key, int index, double value);

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;  // NOLINT

  /** @brief Get the analog key */
  std::string getKey() const;

  /** @brief Get the analog index */
  int getIndex() const;

  /** @brief Get the analgo value */
  double getValue() const;

  /**
   * @brief Equal operator. Does not compare descriptions
   * @param rhs SetAnalogInstruction
   * @return True if equal, otherwise false
   */
  bool operator==(const SetAnalogInstruction& rhs) const;

  /**
   * @brief Not equal operator. Does not compare descriptions
   * @param rhs SetAnalogInstruction
   * @return True if not equal, otherwise false
   */
  bool operator!=(const SetAnalogInstruction& rhs) const;

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Set Analog Instruction" };
  /** @brief The key is used to identify which type of analog to set */
  std::string key_;
  /** @brief The analog index to set */
  int index_{ 0 };
  /** @brief The analog value */
  double value_{ 0 };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(SetAnalogInstruction)
#else
TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning::SetAnalogInstruction);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H

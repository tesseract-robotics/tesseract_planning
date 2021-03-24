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
#include <iostream>
#include <string>
#include <tinyxml2.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class SetAnalogInstruction
{
public:
  SetAnalogInstruction() = default;  // Required for boost serialization do not use
  SetAnalogInstruction(std::string key, int index, double value) : key_(std::move(key)), index_(index), value_(value) {}

  int getType() const { return static_cast<int>(InstructionType::SET_ANALOG_INSTRUCTION); }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(const std::string& prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Set Analog Instruction, Type: " << getType() << ", Key: " << key_ << ", Index: " << index_
              << ", Value: " << value_;
    std::cout << ", Description: " << getDescription() << std::endl;
  }

  /** @brief Get the analog key */
  std::string getKey() const { return key_; }

  /** @brief Get the analog index */
  int getIndex() const { return index_; }

  /** @brief Get the analgo value */
  double getValue() const { return value_; }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
    xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_analog_instruction = doc.NewElement("SetAnalogInstruction");
    xml_analog_instruction->SetAttribute("key", key_.c_str());
    xml_analog_instruction->SetAttribute("index", std::to_string(index_).c_str());
    xml_analog_instruction->SetAttribute("value", std::to_string(value_).c_str());

    tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
    xml_description->SetText(getDescription().c_str());

    xml_analog_instruction->InsertEndChild(xml_description);
    xml_instruction->InsertEndChild(xml_analog_instruction);

    return xml_instruction;
  }

  /**
   * @brief Equal operator. Does not compare descriptions
   * @param rhs SetAnalogInstruction
   * @return True if equal, otherwise false
   */
  bool operator==(const SetAnalogInstruction& rhs) const
  {
    // Check if they are the same
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    bool equal = true;
    equal &= (key_ == rhs.key_);
    equal &= (index_ == rhs.index_);
    equal &= (tesseract_common::almostEqualRelativeAndAbs(value_, rhs.value_, max_diff));
    return equal;
  }

  /**
   * @brief Not equal operator. Does not compare descriptions
   * @param rhs SetAnalogInstruction
   * @return True if not equal, otherwise false
   */
  bool operator!=(const SetAnalogInstruction& rhs) const { return !operator==(rhs); }

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
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& boost::serialization::make_nvp("description", description_);
    ar& boost::serialization::make_nvp("key", key_);
    ar& boost::serialization::make_nvp("index", index_);
    ar& boost::serialization::make_nvp("value", value_);
  }
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(SetAnalogInstruction)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H

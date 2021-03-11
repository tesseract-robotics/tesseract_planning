/**
 * @file set_tool_instruction.h
 * @brief Set tool ID
 *
 * @author Levi Armstrong
 * @date March 11, 2021
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H

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
class SetToolInstruction
{
public:
  SetToolInstruction() = default;  // Required for boost serialization do not use
  SetToolInstruction(int tool_id) : tool_id_(tool_id) {}

  int getType() const { return static_cast<int>(InstructionType::SET_TOOL_INSTRUCTION); }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(const std::string& prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Set Tool Instruction, Type: " << getType() << ", Tool ID: " << tool_id_;
    std::cout << ", Description: " << getDescription() << std::endl;
  }

  /**
   * @brief Get the tool ID
   * @return The tool ID
   */
  int getTool() const { return tool_id_; }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
    xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_tool_instruction = doc.NewElement("SetToolInstruction");
    xml_tool_instruction->SetAttribute("tool_id", std::to_string(tool_id_).c_str());

    tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
    xml_description->SetText(getDescription().c_str());
    xml_tool_instruction->InsertEndChild(xml_description);

    xml_tool_instruction->InsertEndChild(xml_description);
    xml_instruction->InsertEndChild(xml_tool_instruction);

    return xml_instruction;
  }

  /**
   * @brief Equal operator. Does not compare descriptions
   * @param rhs SetToolInstruction
   * @return True if equal, otherwise false
   */
  bool operator==(const SetToolInstruction& rhs) const
  {
    bool equal = true;
    equal &= (tool_id_ == rhs.tool_id_);

    return equal;
  }

  /**
   * @brief Not equal operator. Does not compare descriptions
   * @param rhs SetToolInstruction
   * @return True if not equal, otherwise false
   */
  bool operator!=(const SetToolInstruction& rhs) const { return !operator==(rhs); }

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Set Tool Instruction" };
  int tool_id_{ -1 };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& boost::serialization::make_nvp("description", description_);
    ar& boost::serialization::make_nvp("tool_id", tool_id_);
  }
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(SetToolInstruction)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H

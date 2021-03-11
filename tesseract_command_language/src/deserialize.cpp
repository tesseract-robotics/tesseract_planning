/**
 * @file deserialize.cpp
 * @brief Provide methods for deserialize instructions to xml and deserialization
 *
 * @author Levi Armstrong
 * @date August 17, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
#include <functional>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/impl/deserialize.hpp>

namespace tesseract_planning
{
Waypoint getNullWaypoint(const tinyxml2::XMLElement& /*waypoint_element*/) { return NullWaypoint(); }

Waypoint getCartesianWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* cwp_element = waypoint_element.FirstChildElement("CartesianWaypoint");
  return CartesianWaypoint(*cwp_element);
}

Waypoint getJointWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* jwp_element = waypoint_element.FirstChildElement("JointWaypoint");
  return JointWaypoint(*jwp_element);
}

Waypoint getStateWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* swp_element = waypoint_element.FirstChildElement("StateWaypoint");
  return StateWaypoint(*swp_element);
}

Instruction getNullInstruction(const tinyxml2::XMLElement& /*instruction_element*/) { return NullInstruction(); }

Instruction getPlanInstruction(const tinyxml2::XMLElement& instruction_element, WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* plan_element = instruction_element.FirstChildElement("PlanInstruction");
  if (!plan_element)
    throw std::runtime_error("Missing Child Element MoveInstruction.");

  const tinyxml2::XMLElement* description_element = plan_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = plan_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = plan_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* waypoint_element = plan_element->FirstChildElement("Waypoint");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int plan_type = -1;
  tinyxml2::XMLError status = plan_element->QueryIntAttribute("type", &plan_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse plan instruction type attribute.");

  // Get Waypoint
  int waypoint_type = -1;
  status = waypoint_element->QueryIntAttribute("type", &waypoint_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse plan instruction waypoint type attribute.");

  Waypoint wp = waypoint_parser(*waypoint_element, waypoint_type);

  PlanInstruction plan_instruction(wp, static_cast<PlanInstructionType>(plan_type), profile);

  // Get Manipulator Info
  if (manip_info_element)
    plan_instruction.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  plan_instruction.setDescription(description);
  return plan_instruction;
}

Instruction getMoveInstruction(const tinyxml2::XMLElement& instruction_element, WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* move_element = instruction_element.FirstChildElement("MoveInstruction");
  if (!move_element)
    throw std::runtime_error("Missing Child Element MoveInstruction.");

  const tinyxml2::XMLElement* description_element = move_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = move_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = move_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* waypoint_element = move_element->FirstChildElement("Waypoint");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int move_type = -1;
  tinyxml2::XMLError status = move_element->QueryIntAttribute("type", &move_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse move instruction type attribute.");

  // Get Waypoint
  int waypoint_type = -1;
  status = waypoint_element->QueryIntAttribute("type", &waypoint_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse move instruction waypoint type attribute.");

  Waypoint wp = waypoint_parser(*waypoint_element, waypoint_type);

  MoveInstruction move_instruction(wp, static_cast<MoveInstructionType>(move_type), profile);

  // Get Manipulator Info
  if (manip_info_element)
    move_instruction.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  move_instruction.setDescription(description);
  return move_instruction;
}

Instruction startInstructionfromXML(const tinyxml2::XMLElement& start_instruction_xml,
                                    InstructionParserFn instruction_parser,
                                    WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* instruction_xml = start_instruction_xml.FirstChildElement("Instruction");
  if (!instruction_xml)
    throw std::runtime_error("Missing Child Element Instruction.");

  int type = -1;
  tinyxml2::XMLError status = instruction_xml->QueryIntAttribute("type", &type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse start instruction type attribute.");

  return instruction_parser(*instruction_xml, type, waypoint_parser);
}

Instruction getWaitInstruction(const tinyxml2::XMLElement& instruction_element)
{
  const tinyxml2::XMLElement* wait_element = instruction_element.FirstChildElement("WaitInstruction");
  if (!wait_element)
    throw std::runtime_error("Missing Child Element WaitInstruction.");

  const tinyxml2::XMLElement* description_element = wait_element->FirstChildElement("Description");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  int wait_type{ -1 };
  tinyxml2::XMLError status = wait_element->QueryIntAttribute("type", &wait_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse wait instruction type attribute.");

  if (wait_type == static_cast<int>(WaitInstructionType::TIME))
  {
    double wait_time{ 0 };
    tinyxml2::XMLError status = wait_element->QueryDoubleAttribute("time", &wait_time);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Failed to parse wait instruction time attribute.");

    WaitInstruction wait_instruction(wait_time);
    wait_instruction.setDescription(description);
    return wait_instruction;
  }

  int wait_io{ -1 };
  status = wait_element->QueryIntAttribute("io", &wait_io);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse wait instruction io attribute.");
  WaitInstruction wait_instruction(static_cast<WaitInstructionType>(wait_type), wait_io);
  wait_instruction.setDescription(description);
  return wait_instruction;
}

Instruction getTimerInstruction(const tinyxml2::XMLElement& instruction_element)
{
  const tinyxml2::XMLElement* timer_element = instruction_element.FirstChildElement("TimerInstruction");
  if (!timer_element)
    throw std::runtime_error("Missing Child Element TimerInstruction.");

  const tinyxml2::XMLElement* description_element = timer_element->FirstChildElement("Description");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  int timer_type{ -1 };
  tinyxml2::XMLError status = timer_element->QueryIntAttribute("type", &timer_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse timer instruction type attribute.");

  double timer_time{ 0 };
  status = timer_element->QueryDoubleAttribute("time", &timer_time);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse timer instruction time attribute.");

  int timer_io{ -1 };
  status = timer_element->QueryIntAttribute("io", &timer_io);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse timer instruction io attribute.");

  TimerInstruction timer_instruction(static_cast<TimerInstructionType>(timer_type), timer_time, timer_io);
  timer_instruction.setDescription(description);

  return timer_instruction;
}

Instruction getSetToolInstruction(const tinyxml2::XMLElement& instruction_element)
{
  const tinyxml2::XMLElement* timer_element = instruction_element.FirstChildElement("SetToolInstruction");
  if (!timer_element)
    throw std::runtime_error("Missing Child Element SetToolInstruction.");

  const tinyxml2::XMLElement* description_element = timer_element->FirstChildElement("Description");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  int tool_id{ -1 };
  tinyxml2::XMLError status = timer_element->QueryIntAttribute("tool_id", &tool_id);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse set tool instruction tool_id attribute.");

  SetToolInstruction set_tool_instruction(tool_id);
  set_tool_instruction.setDescription(description);

  return set_tool_instruction;
}

Instruction getCompositeInstruction(const tinyxml2::XMLElement& instruction_element,
                                    InstructionParserFn instruction_parser,
                                    WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* composite_element = instruction_element.FirstChildElement("CompositeInstruction");
  if (!composite_element)
    throw std::runtime_error("Missing Child Element CompositeInstruction");

  const tinyxml2::XMLElement* description_element = composite_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = composite_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = composite_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* start_instruction_element = composite_element->FirstChildElement("StartInstruction");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int order_type = -1;
  tinyxml2::XMLError status = composite_element->QueryIntAttribute("order", &order_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

  CompositeInstruction composite(profile, CompositeInstructionOrder(order_type));

  // Get Manipulator Info
  if (manip_info_element)
    composite.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  composite.setDescription(description);

  if (start_instruction_element)
    composite.setStartInstruction(
        startInstructionfromXML(*start_instruction_element, instruction_parser, waypoint_parser));

  for (const tinyxml2::XMLElement* child_instruction = composite_element->FirstChildElement("Instruction");
       child_instruction;
       child_instruction = child_instruction->NextSiblingElement("Instruction"))
  {
    int type = -1;
    status = child_instruction->QueryIntAttribute("type", &type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

    composite.push_back(instruction_parser(*child_instruction, type, waypoint_parser));
  }

  return composite;
}

Instruction InstructionParser(const tinyxml2::XMLElement& xml_element, int type, WaypointParserFn waypoint_parser)
{
  switch (type)
  {
    case static_cast<int>(InstructionType::NULL_INSTRUCTION):
    {
      return getNullInstruction(xml_element);
    }
    case static_cast<int>(InstructionType::PLAN_INSTRUCTION):
    {
      return getPlanInstruction(xml_element, waypoint_parser);
    }
    case static_cast<int>(InstructionType::MOVE_INSTRUCTION):
    {
      return getMoveInstruction(xml_element, waypoint_parser);
    }
    case static_cast<int>(InstructionType::WAIT_INSTRUCTION):
    {
      return getWaitInstruction(xml_element);
    }
    case static_cast<int>(InstructionType::TIMER_INSTRUCTION):
    {
      return getTimerInstruction(xml_element);
    }
    case static_cast<int>(InstructionType::SET_TOOL_INSTRUCTION):
    {
      return getSetToolInstruction(xml_element);
    }
    case static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION):
    {
      return getCompositeInstruction(xml_element, InstructionParser, waypoint_parser);
    }
    default:
    {
      throw std::runtime_error("Unsupported Instruction Type!");
    }
  }
}

Instruction defaultInstructionParser(const tinyxml2::XMLElement& xml_element, int type)
{
  return InstructionParser(xml_element, type, defaultWaypointParser);
}

Waypoint defaultWaypointParser(const tinyxml2::XMLElement& xml_element, int type)
{
  switch (type)
  {
    case static_cast<int>(WaypointType::NULL_WAYPOINT):
    {
      return getNullWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::CARTESIAN_WAYPOINT):
    {
      return getCartesianWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      return getJointWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::STATE_WAYPOINT):
    {
      return getStateWaypoint(xml_element);
    }
    default:
    {
      throw std::runtime_error("Unsupported Waypoint Type!");
    }
  }
}

// Explicit specialization
// Instruction
template Instruction fromXMLDocument<Instruction>(const tinyxml2::XMLDocument& xml_doc,
                                                  std::function<Instruction(const tinyxml2::XMLElement&, int)>);

template Instruction fromXMLFile<Instruction>(const std::string& file_path,
                                              std::function<Instruction(const tinyxml2::XMLElement&, int)> parser);

template Instruction fromXMLString<Instruction>(const std::string& xml_string,
                                                std::function<Instruction(const tinyxml2::XMLElement&, int)> parser);

template <>
struct XMLElementName<Instruction>
{
  static constexpr char value[] = "Instruction";
};

// Waypoint
template Waypoint fromXMLDocument<Waypoint>(const tinyxml2::XMLDocument& xml_doc,
                                            std::function<Waypoint(const tinyxml2::XMLElement&, int)> parser);

template Waypoint fromXMLFile<Waypoint>(const std::string& file_path,
                                        std::function<Waypoint(const tinyxml2::XMLElement&, int)> parser);

template Waypoint fromXMLString<Waypoint>(const std::string& xml_string,
                                          std::function<Waypoint(const tinyxml2::XMLElement&, int)> parser);

template <>
struct XMLElementName<Waypoint>
{
  static constexpr char value[] = "Waypoint";
};
}  // namespace tesseract_planning

/**
 * @file deserialize.hpp
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
#ifndef TESSERACT_COMMAND_LANGUAGE_DESERIALIZE_HPP
#define TESSERACT_COMMAND_LANGUAGE_DESERIALIZE_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <fstream>
#include <array>
#include <console_bridge/console.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/deserialize.h>

namespace tesseract_planning
{
/** @brief Used to lookup the XML element name for a type.
 *
 * Example Usage:
 * XMLElementForType<Instruction> element_name;
 * std::string string_name = element_name.value;*/
template <class SerializableType>
struct XMLElementName;

template <typename SerializableType>
SerializableType fromXMLElement(const tinyxml2::XMLElement* cl_xml,
                                std::function<SerializableType(const tinyxml2::XMLElement&, int)> parser)
{
  std::array<int, 3> version;
  std::string version_string;
  tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(cl_xml, "version", version_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Error parsing robot attribute 'version'");

  if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, version_string, boost::is_any_of("."), boost::token_compress_on);
    if (tokens.size() < 2 || tokens.size() > 3 || !tesseract_common::isNumeric(tokens))
      throw std::runtime_error("fromXML: Error parsing robot attribute 'version'");

    tesseract_common::toNumeric<int>(tokens[0], version[0]);
    tesseract_common::toNumeric<int>(tokens[1], version[1]);
    if (tokens.size() == 3)
      tesseract_common::toNumeric<int>(tokens[2], version[2]);
    else
      version[2] = 0;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("No version number was provided so latest parser will be used.");
  }

  // Looks up the name of the child element
  XMLElementName<SerializableType> element_name;

  const tinyxml2::XMLElement* xml_element = cl_xml->FirstChildElement(element_name.value);
  if (!xml_element)
    throw std::runtime_error("fromXML: Could not find the " + std::string(element_name.value) +
                             " element in the xml file.");

  int type = 0;
  status = xml_element->QueryIntAttribute("type", &type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

  return parser(*xml_element, type);
}

template <typename SerializableType>
SerializableType fromXMLDocument(const tinyxml2::XMLDocument& xml_doc,
                                 std::function<SerializableType(const tinyxml2::XMLElement&, int)> parser)
{
  const tinyxml2::XMLElement* cl_xml = xml_doc.FirstChildElement("CommandLanguage");
  if (!cl_xml)
    throw std::runtime_error("Could not find the 'CommandLanguage' element in the XML file");

  return fromXMLElement<SerializableType>(cl_xml, parser);
}

template <typename SerializableType>
SerializableType fromXMLFile(const std::string& file_path,
                             std::function<SerializableType(const tinyxml2::XMLElement&, int)> parser)
{
  // get the entire file
  std::string xml_string;
  std::fstream xml_file(file_path.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while (xml_file.good())
    {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return fromXMLString<SerializableType>(xml_string, parser);
  }

  throw std::runtime_error("Could not open file " + file_path + "for parsing.");
}

template <typename SerializableType>
SerializableType fromXMLString(const std::string& xml_string,
                               std::function<SerializableType(const tinyxml2::XMLElement&, int)> parser)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xml_string.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
    throw std::runtime_error("Could not parse the XML File.");

  return fromXMLDocument<SerializableType>(xml_doc, parser);
}
}  // namespace tesseract_planning
#endif

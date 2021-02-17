/**
 * @file serialize.hpp
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SERIALIZE_HPP
#define TESSERACT_COMMAND_LANGUAGE_SERIALIZE_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <fstream>
#include <array>
#include <console_bridge/console.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/core/waypoint.h>

namespace
{
inline const std::array<int, 3> VERSION{ { 1, 0, 0 } };
inline const Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
}  // namespace

namespace tesseract_planning
{
template <typename SerializableType>
std::shared_ptr<tinyxml2::XMLDocument> toXMLDocument(const SerializableType& input)
{
  auto doc = std::make_shared<tinyxml2::XMLDocument>();
  tinyxml2::XMLElement* xml_root = doc->NewElement("CommandLanguage");
  xml_root->SetAttribute(
      "version",
      (std::to_string(VERSION[0]) + "." + std::to_string(VERSION[1]) + "." + std::to_string(VERSION[2])).c_str());

  tinyxml2::XMLElement* xml_instruction = input.toXML(*doc);
  xml_root->InsertEndChild(xml_instruction);
  doc->InsertFirstChild(xml_root);
  return doc;
}

template <typename SerializableType>
bool toXMLFile(const SerializableType& input, const std::string& file_path)
{
  std::shared_ptr<tinyxml2::XMLDocument> doc = toXMLDocument<SerializableType>(input);
  tinyxml2::XMLError status = doc->SaveFile(file_path.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Failed to save XML File: %s", file_path.c_str());
    return false;
  }

  return true;
}

template <typename SerializableType>
std::string toXMLString(const SerializableType& input)
{
  std::shared_ptr<tinyxml2::XMLDocument> doc = toXMLDocument<SerializableType>(input);
  tinyxml2::XMLPrinter printer;
  doc->Print(&printer);
  return std::string(printer.CStr());
}
}  // namespace tesseract_planning
#endif

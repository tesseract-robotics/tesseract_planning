/**
 * @file serialize.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <fstream>
#include <array>
#include <console_bridge/console.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/impl/serialize.hpp>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/core/waypoint.h>

namespace tesseract_planning
{
template std::shared_ptr<tinyxml2::XMLDocument> toXMLDocument<Instruction>(const Instruction& input);
template bool toXMLFile<Instruction>(const Instruction& instruction, const std::string& file_path);
template std::string toXMLString<Instruction>(const Instruction& instruction);

template std::shared_ptr<tinyxml2::XMLDocument> toXMLDocument<Waypoint>(const Waypoint& input);
template bool toXMLFile<Waypoint>(const Waypoint& input, const std::string& file_path);
template std::string toXMLString<Waypoint>(const Waypoint& input);
}  // namespace tesseract_planning

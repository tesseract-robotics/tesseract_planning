/**
 * @file deserialize.h
 * @brief Provide methods for deserialize ompl plans to xml
 *
 * @author Tyler Marr
 * @date August 24, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_DESERIALIZE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_DESERIALIZE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract_planning
{
class OMPLDefaultPlanProfile;

OMPLDefaultPlanProfile omplPlanParser(const tinyxml2::XMLElement& xml_element);

OMPLDefaultPlanProfile omplPlanFromXMLElement(const tinyxml2::XMLElement* profile_xml);

OMPLDefaultPlanProfile omplPlanFromXMLDocument(const tinyxml2::XMLDocument& xml_doc);

OMPLDefaultPlanProfile omplPlanFromXMLFile(const std::string& file_path);

OMPLDefaultPlanProfile omplPlanFromXMLString(const std::string& xml_string);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_DESERIALIZE_H

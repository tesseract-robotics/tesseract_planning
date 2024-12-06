/**
 * @file trajopt_collision_config.cpp
 * @brief TrajOpt collision configuration settings
 *
 * @author Tyler Marr
 * @date November 2, 2023
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
#include <stdexcept>
#include <tinyxml2.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/eigen_serialization.h>

namespace tesseract_planning
{
CartesianWaypointConfig::CartesianWaypointConfig(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* enabled_element = xml_element.FirstChildElement("Enabled");
  const tinyxml2::XMLElement* use_tolerance_override_element = xml_element.FirstChildElement("UseToleranceOverride");
  const tinyxml2::XMLElement* lower_tolerance_element = xml_element.FirstChildElement("LowerTolerance");
  const tinyxml2::XMLElement* upper_tolerance_element = xml_element.FirstChildElement("UpperTolerance");
  const tinyxml2::XMLElement* coefficients_element = xml_element.FirstChildElement("Coefficients");

  if (enabled_element == nullptr)
    throw std::runtime_error("CartesianWaypointConfig: Must have Enabled element.");
  int status = enabled_element->QueryBoolText(&enabled);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("CartesianWaypointConfig: Error parsing Enabled string");

  if (use_tolerance_override_element != nullptr)
  {
    status = use_tolerance_override_element->QueryBoolText(&use_tolerance_override);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CartesianWaypointConfig: Error parsing UseToleranceOverride string");
  }

  if (lower_tolerance_element != nullptr)
  {
    std::vector<std::string> lower_tolerance_tokens;
    std::string lower_tolerance_string;
    status = tesseract_common::QueryStringText(lower_tolerance_element, lower_tolerance_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CartesianWaypointConfig: Error parsing LowerTolerance string");

    boost::split(lower_tolerance_tokens, lower_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(lower_tolerance_tokens))
      throw std::runtime_error("CartesianWaypointConfig: LowerTolerance are not all numeric values.");

    lower_tolerance.resize(static_cast<long>(lower_tolerance_tokens.size()));
    for (std::size_t i = 0; i < lower_tolerance_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(lower_tolerance_tokens[i], lower_tolerance[static_cast<long>(i)]);
  }

  if (upper_tolerance_element != nullptr)
  {
    std::vector<std::string> upper_tolerance_tokens;
    std::string upper_tolerance_string;
    status = tesseract_common::QueryStringText(upper_tolerance_element, upper_tolerance_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CartesianWaypointConfig: Error parsing UpperTolerance string");

    boost::split(upper_tolerance_tokens, upper_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(upper_tolerance_tokens))
      throw std::runtime_error("CartesianWaypointConfig: UpperTolerance are not all numeric values.");

    upper_tolerance.resize(static_cast<long>(upper_tolerance_tokens.size()));
    for (std::size_t i = 0; i < upper_tolerance_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(upper_tolerance_tokens[i], upper_tolerance[static_cast<long>(i)]);
  }

  if (coefficients_element != nullptr)
  {
    std::vector<std::string> coefficients_tokens;
    std::string coefficients_string;
    status = tesseract_common::QueryStringText(coefficients_element, coefficients_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CartesianWaypointConfig: Error parsing Coefficients string");

    boost::split(coefficients_tokens, coefficients_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(coefficients_tokens))
      throw std::runtime_error("CartesianWaypointConfig: Coefficients are not all numeric values.");

    coeff.resize(static_cast<long>(coefficients_tokens.size()));
    for (std::size_t i = 0; i < coefficients_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(coefficients_tokens[i], coeff[static_cast<long>(i)]);
  }
}

tinyxml2::XMLElement* CartesianWaypointConfig::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  tinyxml2::XMLElement* xml_cartesian_waypoint_config = doc.NewElement("CartesianWaypointConfig");

  tinyxml2::XMLElement* xml_enabled = doc.NewElement("Enabled");
  xml_enabled->SetText(enabled);
  xml_cartesian_waypoint_config->InsertEndChild(xml_enabled);

  tinyxml2::XMLElement* xml_use_tolerance_override = doc.NewElement("UseToleranceOverride");
  xml_use_tolerance_override->SetText(use_tolerance_override);
  xml_cartesian_waypoint_config->InsertEndChild(xml_use_tolerance_override);

  if (lower_tolerance.size() > 0)
  {
    tinyxml2::XMLElement* xml_lower_tolerance = doc.NewElement("LowerTolerance");
    std::stringstream lower_tolerance_ss;
    lower_tolerance_ss << lower_tolerance.format(eigen_format);
    xml_lower_tolerance->SetText(lower_tolerance_ss.str().c_str());
    xml_cartesian_waypoint_config->InsertEndChild(xml_lower_tolerance);
  }

  if (upper_tolerance.size() > 0)
  {
    tinyxml2::XMLElement* xml_upper_tolerance = doc.NewElement("UpperTolerance");
    std::stringstream upper_tolerance_ss;
    upper_tolerance_ss << upper_tolerance.format(eigen_format);
    xml_upper_tolerance->SetText(upper_tolerance_ss.str().c_str());
    xml_cartesian_waypoint_config->InsertEndChild(xml_upper_tolerance);
  }

  tinyxml2::XMLElement* xml_coeff = doc.NewElement("Coefficients");
  std::stringstream coeff_ss;
  coeff_ss << coeff.format(eigen_format);
  xml_coeff->SetText(coeff_ss.str().c_str());
  xml_cartesian_waypoint_config->InsertEndChild(xml_coeff);

  return xml_cartesian_waypoint_config;
}

template <class Archive>
void CartesianWaypointConfig::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(enabled);
  ar& BOOST_SERIALIZATION_NVP(use_tolerance_override);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(coeff);
}

JointWaypointConfig::JointWaypointConfig(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* enabled_element = xml_element.FirstChildElement("Enabled");
  const tinyxml2::XMLElement* use_tolerance_override_element = xml_element.FirstChildElement("UseToleranceOverride");
  const tinyxml2::XMLElement* lower_tolerance_element = xml_element.FirstChildElement("LowerTolerance");
  const tinyxml2::XMLElement* upper_tolerance_element = xml_element.FirstChildElement("UpperTolerance");
  const tinyxml2::XMLElement* coefficients_element = xml_element.FirstChildElement("Coefficients");

  if (enabled_element == nullptr)
    throw std::runtime_error("JointWaypointConfig: Must have Enabled element.");

  int status = enabled_element->QueryBoolText(&enabled);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("JointWaypointConfig: Error parsing Enabled string");

  if (use_tolerance_override_element != nullptr)
  {
    status = use_tolerance_override_element->QueryBoolText(&use_tolerance_override);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypointConfig: Error parsing UseToleranceOverride string");
  }

  if (lower_tolerance_element != nullptr)
  {
    std::vector<std::string> lower_tolerance_tokens;
    std::string lower_tolerance_string;
    status = tesseract_common::QueryStringText(lower_tolerance_element, lower_tolerance_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypointConfig: Error parsing LowerTolerance string");

    boost::split(lower_tolerance_tokens, lower_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(lower_tolerance_tokens))
      throw std::runtime_error("JointWaypointConfig: LowerTolerance are not all numeric values.");

    lower_tolerance.resize(static_cast<long>(lower_tolerance_tokens.size()));
    for (std::size_t i = 0; i < lower_tolerance_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(lower_tolerance_tokens[i], lower_tolerance[static_cast<long>(i)]);
  }

  if (upper_tolerance_element != nullptr)
  {
    std::vector<std::string> upper_tolerance_tokens;
    std::string upper_tolerance_string;
    status = tesseract_common::QueryStringText(upper_tolerance_element, upper_tolerance_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypointConfig: Error parsing UpperTolerance string");

    boost::split(upper_tolerance_tokens, upper_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(upper_tolerance_tokens))
      throw std::runtime_error("JointWaypointConfig: UpperTolerance are not all numeric values.");

    upper_tolerance.resize(static_cast<long>(upper_tolerance_tokens.size()));
    for (std::size_t i = 0; i < upper_tolerance_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(upper_tolerance_tokens[i], upper_tolerance[static_cast<long>(i)]);
  }

  if (coefficients_element != nullptr)
  {
    std::vector<std::string> coefficients_tokens;
    std::string coefficients_string;
    status = tesseract_common::QueryStringText(coefficients_element, coefficients_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypointConfig: Error parsing Coefficients string");

    boost::split(coefficients_tokens, coefficients_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(coefficients_tokens))
      throw std::runtime_error("JointWaypointConfig: Coefficients are not all numeric values.");

    coeff.resize(static_cast<long>(coefficients_tokens.size()));
    for (std::size_t i = 0; i < coefficients_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(coefficients_tokens[i], coeff[static_cast<long>(i)]);
  }
}

tinyxml2::XMLElement* JointWaypointConfig::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  tinyxml2::XMLElement* xml_cartesian_waypoint_config = doc.NewElement("JointWaypointConfig");

  tinyxml2::XMLElement* xml_enabled = doc.NewElement("Enabled");
  xml_enabled->SetText(enabled);
  xml_cartesian_waypoint_config->InsertEndChild(xml_enabled);

  tinyxml2::XMLElement* xml_use_tolerance_override = doc.NewElement("UseToleranceOverride");
  xml_use_tolerance_override->SetText(use_tolerance_override);
  xml_cartesian_waypoint_config->InsertEndChild(xml_use_tolerance_override);

  if (lower_tolerance.size() > 0)
  {
    tinyxml2::XMLElement* xml_lower_tolerance = doc.NewElement("LowerTolerance");
    std::stringstream lower_tolerance_ss;
    lower_tolerance_ss << lower_tolerance.format(eigen_format);
    xml_lower_tolerance->SetText(lower_tolerance_ss.str().c_str());
    xml_cartesian_waypoint_config->InsertEndChild(xml_lower_tolerance);
  }

  if (upper_tolerance.size() > 0)
  {
    tinyxml2::XMLElement* xml_upper_tolerance = doc.NewElement("UpperTolerance");
    std::stringstream upper_tolerance_ss;
    upper_tolerance_ss << upper_tolerance.format(eigen_format);
    xml_upper_tolerance->SetText(upper_tolerance_ss.str().c_str());
    xml_cartesian_waypoint_config->InsertEndChild(xml_upper_tolerance);
  }

  tinyxml2::XMLElement* xml_coeff = doc.NewElement("Coefficients");
  std::stringstream coeff_ss;
  coeff_ss << coeff.format(eigen_format);
  xml_coeff->SetText(coeff_ss.str().c_str());
  xml_cartesian_waypoint_config->InsertEndChild(xml_coeff);

  return xml_cartesian_waypoint_config;
}

template <class Archive>
void JointWaypointConfig::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(enabled);
  ar& BOOST_SERIALIZATION_NVP(use_tolerance_override);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(coeff);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypointConfig)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointConfig)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::JointWaypointConfig)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointConfig)

/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Samantha Smith
 * @date July 14, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H
#define TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>

namespace YAML
{
//=========================== Fix State Bounds Settings Enum ===========================
template <>
struct convert<tesseract_planning::FixStateBoundsProfile::Settings>
{
  static Node encode(const tesseract_planning::FixStateBoundsProfile::Settings& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract_planning::FixStateBoundsProfile::Settings, std::string> m = {
      { tesseract_planning::FixStateBoundsProfile::Settings::START_ONLY, "START_ONLY" },
      { tesseract_planning::FixStateBoundsProfile::Settings::END_ONLY, "END_ONLY" },
      { tesseract_planning::FixStateBoundsProfile::Settings::ALL, "ALL" },
      { tesseract_planning::FixStateBoundsProfile::Settings::DISABLED, "DISABLED" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract_planning::FixStateBoundsProfile::Settings& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract_planning::FixStateBoundsProfile::Settings> inv = {
      { "START_ONLY", tesseract_planning::FixStateBoundsProfile::Settings::START_ONLY },
      { "END_ONLY", tesseract_planning::FixStateBoundsProfile::Settings::END_ONLY },
      { "ALL", tesseract_planning::FixStateBoundsProfile::Settings::ALL },
      { "DISABLED", tesseract_planning::FixStateBoundsProfile::Settings::DISABLED }
    };
    // LCOV_EXCL_STOP

    if (!node.IsScalar())
      return false;

    auto it = inv.find(node.Scalar());
    if (it == inv.end())
      return false;

    rhs = it->second;
    return true;
  }
};

//=========================== Fix State Collision Settings Enum ===========================
template <>
struct convert<tesseract_planning::FixStateCollisionProfile::Settings>
{
  static Node encode(const tesseract_planning::FixStateCollisionProfile::Settings& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract_planning::FixStateCollisionProfile::Settings, std::string> m = {
      { tesseract_planning::FixStateCollisionProfile::Settings::START_ONLY, "START_ONLY" },
      { tesseract_planning::FixStateCollisionProfile::Settings::END_ONLY, "END_ONLY" },
      { tesseract_planning::FixStateCollisionProfile::Settings::INTERMEDIATE_ONLY, "INTERMEDIATE_ONLY" },
      { tesseract_planning::FixStateCollisionProfile::Settings::ALL, "ALL" },
      { tesseract_planning::FixStateCollisionProfile::Settings::ALL_EXCEPT_START, "ALL_EXCEPT_START" },
      { tesseract_planning::FixStateCollisionProfile::Settings::ALL_EXCEPT_END, "ALL_EXCEPT_END" },
      { tesseract_planning::FixStateCollisionProfile::Settings::DISABLED, "DISABLED" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract_planning::FixStateCollisionProfile::Settings& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract_planning::FixStateCollisionProfile::Settings> inv = {
      { "START_ONLY", tesseract_planning::FixStateCollisionProfile::Settings::START_ONLY },
      { "END_ONLY", tesseract_planning::FixStateCollisionProfile::Settings::END_ONLY },
      { "INTERMEDIATE_ONLY", tesseract_planning::FixStateCollisionProfile::Settings::INTERMEDIATE_ONLY },
      { "ALL", tesseract_planning::FixStateCollisionProfile::Settings::ALL },
      { "ALL_EXCEPT_START", tesseract_planning::FixStateCollisionProfile::Settings::ALL_EXCEPT_START },
      { "ALL_EXCEPT_END", tesseract_planning::FixStateCollisionProfile::Settings::ALL_EXCEPT_END },
      { "DISABLED", tesseract_planning::FixStateCollisionProfile::Settings::DISABLED }
    };
    // LCOV_EXCL_STOP

    if (!node.IsScalar())
      return false;

    auto it = inv.find(node.Scalar());
    if (it == inv.end())
      return false;

    rhs = it->second;
    return true;
  }
};

//=========================== Fix State Collision CorrectionMethod Enum ===========================
template <>
struct convert<tesseract_planning::FixStateCollisionProfile::CorrectionMethod>
{
  static Node encode(const tesseract_planning::FixStateCollisionProfile::CorrectionMethod& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract_planning::FixStateCollisionProfile::CorrectionMethod, std::string> m = {
      { tesseract_planning::FixStateCollisionProfile::CorrectionMethod::NONE, "NONE" },
      { tesseract_planning::FixStateCollisionProfile::CorrectionMethod::TRAJOPT, "TRAJOPT" },
      { tesseract_planning::FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER, "RANDOM_SAMPLER" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract_planning::FixStateCollisionProfile::CorrectionMethod& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract_planning::FixStateCollisionProfile::CorrectionMethod> inv = {
      { "NONE", tesseract_planning::FixStateCollisionProfile::CorrectionMethod::NONE },
      { "TRAJOPT", tesseract_planning::FixStateCollisionProfile::CorrectionMethod::TRAJOPT },
      { "RANDOM_SAMPLER", tesseract_planning::FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER }
    };
    // LCOV_EXCL_STOP

    if (!node.IsScalar())
      return false;

    auto it = inv.find(node.Scalar());
    if (it == inv.end())
      return false;

    rhs = it->second;
    return true;
  }
};

}  // namespace YAML

#endif  // TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H

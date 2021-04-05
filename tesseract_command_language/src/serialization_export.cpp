/**
 * @file serialization_export.cpp
 * @brief Provides boost exports for waypoint and instruction implementation for serialization
 *
 * @author Levi Armstrong
 * @date February 24, 2021
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/serialization.h>
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/command_language.h>

// template struct tesseract_planning::detail_waypoint::WaypointInner<tesseract_planning::CartesianWaypoint>;

// BOOST_EXPORT_CLASS_IMPLEMENT(tesseract_planning::detail_waypoint::WaypointInner<tesseract_planning::CartesianWaypoint>);

// template tesseract_planning::CartesianWaypoint::serialize(boost::archive::xml_oarchive& ar, const unsigned int
// version); template tesseract_planning::CartesianWaypoint::serialize(boost::archive::xml_iarchive& ar, const unsigned
// int version);

// TESSERACT_WAYPOINT_EXPORT(tesseract_planning::JointWaypoint);      // NOLINT
// TESSERACT_WAYPOINT_EXPORT(tesseract_planning::CartesianWaypoint);  // NOLINT
// TESSERACT_WAYPOINT_EXPORT(tesseract_planning::StateWaypoint);      // NOLINT

// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::MoveInstruction);       // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::PlanInstruction);       // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::WaitInstruction);       // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::TimerInstruction);      // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::SetToolInstruction);    // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::SetAnalogInstruction);  // NOLINT
// TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::CompositeInstruction);  // NOLINT

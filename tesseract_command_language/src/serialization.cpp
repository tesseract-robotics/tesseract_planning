/**
 * @file serialization.h
 * @brief Provides boost exports for waypoint and instruction implementation for serialization
 * Also includes utility function
 *   - toArchiveStringXML
 *   - toArchiveFileXML
 *   - fromArchiveStringXML
 *   - fromArchiveFileXML
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

#include <tesseract_command_language/serialization.h>

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/command_language.h>

TESSERACT_WAYPOINT_EXPORT(tesseract_planning::NullWaypoint);       // NOLINT
TESSERACT_WAYPOINT_EXPORT(tesseract_planning::JointWaypoint);      // NOLINT
TESSERACT_WAYPOINT_EXPORT(tesseract_planning::CartesianWaypoint);  // NOLINT
TESSERACT_WAYPOINT_EXPORT(tesseract_planning::StateWaypoint);      // NOLINT

TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::NullInstruction);       // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::MoveInstruction);       // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::PlanInstruction);       // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::WaitInstruction);       // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::TimerInstruction);      // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::SetToolInstruction);    // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::SetAnalogInstruction);  // NOLINT
TESSERACT_INSTRUCTION_EXPORT(tesseract_planning::CompositeInstruction);  // NOLINT

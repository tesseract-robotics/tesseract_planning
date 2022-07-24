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
#include <tesseract_common/serialization.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/composite_instruction.h>

template std::string
tesseract_common::Serialization::toArchiveStringXML(const tesseract_planning::InstructionPoly& archive_type,
                                                    const std::string& name);
template std::string
tesseract_common::Serialization::toArchiveStringXML(const tesseract_planning::WaypointPoly& archive_type,
                                                    const std::string& name);
template std::string
tesseract_common::Serialization::toArchiveStringXML(const tesseract_planning::CompositeInstruction& archive_type,
                                                    const std::string& name);

template bool tesseract_common::Serialization::toArchiveFileXML(const tesseract_planning::InstructionPoly& archive_type,
                                                                const std::string& file_path,
                                                                const std::string& name);
template bool tesseract_common::Serialization::toArchiveFileXML(const tesseract_planning::WaypointPoly& archive_type,
                                                                const std::string& file_path,
                                                                const std::string& name);
template bool
tesseract_common::Serialization::toArchiveFileXML(const tesseract_planning::CompositeInstruction& archive_type,
                                                  const std::string& file_path,
                                                  const std::string& name);

template bool
tesseract_common::Serialization::toArchiveFileBinary(const tesseract_planning::InstructionPoly& archive_type,
                                                     const std::string& file_path,
                                                     const std::string& name);
template bool tesseract_common::Serialization::toArchiveFileBinary(const tesseract_planning::WaypointPoly& archive_type,
                                                                   const std::string& file_path,
                                                                   const std::string& name);
template bool
tesseract_common::Serialization::toArchiveFileBinary(const tesseract_planning::CompositeInstruction& archive_type,
                                                     const std::string& file_path,
                                                     const std::string& name);

template tesseract_planning::InstructionPoly
tesseract_common::Serialization::fromArchiveStringXML(const std::string& archive_xml);
template tesseract_planning::WaypointPoly
tesseract_common::Serialization::fromArchiveStringXML(const std::string& archive_xml);
template tesseract_planning::CompositeInstruction
tesseract_common::Serialization::fromArchiveStringXML(const std::string& archive_xml);

template tesseract_planning::InstructionPoly
tesseract_common::Serialization::fromArchiveFileXML(const std::string& file_path);
template tesseract_planning::WaypointPoly
tesseract_common::Serialization::fromArchiveFileXML(const std::string& file_path);
template tesseract_planning::CompositeInstruction
tesseract_common::Serialization::fromArchiveFileXML(const std::string& file_path);

template tesseract_planning::InstructionPoly
tesseract_common::Serialization::fromArchiveFileBinary(const std::string& file_path);
template tesseract_planning::WaypointPoly
tesseract_common::Serialization::fromArchiveFileBinary(const std::string& file_path);
template tesseract_planning::CompositeInstruction
tesseract_common::Serialization::fromArchiveFileBinary(const std::string& file_path);

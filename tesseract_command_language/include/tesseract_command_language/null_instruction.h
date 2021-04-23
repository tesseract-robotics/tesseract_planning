/**
 * @file null_instruction.h
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

#ifndef TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>

namespace tesseract_planning
{
class NullInstruction
{
public:
  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;

  bool operator==(const NullInstruction& rhs) const;
  bool operator!=(const NullInstruction& rhs) const;

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Null Instruction" };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/);  // NOLINT
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(NullInstruction)
#else
TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning::NullInstruction);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H

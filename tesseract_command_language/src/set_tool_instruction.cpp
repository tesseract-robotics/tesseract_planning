/**
 * @file set_tool_instruction.cpp
 * @brief Set tool ID
 *
 * @author Levi Armstrong
 * @date March 11, 2021
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
#include <iostream>
#include <string>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
SetToolInstruction::SetToolInstruction(int tool_id) : uuid_(boost::uuids::random_generator()()), tool_id_(tool_id) {}

const boost::uuids::uuid& SetToolInstruction::getUUID() const { return uuid_; }
void SetToolInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("SetToolInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void SetToolInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& SetToolInstruction::getParentUUID() const { return parent_uuid_; }
void SetToolInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

const std::string& SetToolInstruction::getDescription() const { return description_; }

void SetToolInstruction::setDescription(const std::string& description) { description_ = description; }

void SetToolInstruction::print(const std::string& prefix) const  // NOLINT
{
  std::cout << prefix + "Set Tool Instruction, Tool ID: " << tool_id_;
  std::cout << ", Description: " << getDescription() << std::endl;
}

int SetToolInstruction::getTool() const { return tool_id_; }

bool SetToolInstruction::operator==(const SetToolInstruction& rhs) const
{
  bool equal = true;
  equal &= (tool_id_ == rhs.tool_id_);

  return equal;
}
// LCOV_EXCL_START
bool SetToolInstruction::operator!=(const SetToolInstruction& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void SetToolInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("tool_id", tool_id_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SetToolInstruction)
TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::SetToolInstruction);

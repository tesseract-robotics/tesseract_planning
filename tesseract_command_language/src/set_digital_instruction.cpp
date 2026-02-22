/**
 * @file set_digital_instruction.cpp
 * @brief Set Digital Instruction
 *
 * @author Levi Armstrong
 * @date March 23, 2025
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <boost/uuid/uuid_generators.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/set_digital_instruction.h>
#include <tesseract_common/utils.h>

namespace tesseract::command_language
{
SetDigitalInstruction::SetDigitalInstruction(std::string key, int index, bool value)
  : uuid_(boost::uuids::random_generator()()), key_(std::move(key)), index_(index), value_(value)
{
}

const boost::uuids::uuid& SetDigitalInstruction::getUUID() const { return uuid_; }
void SetDigitalInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("SetDigitalInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void SetDigitalInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& SetDigitalInstruction::getParentUUID() const { return parent_uuid_; }
void SetDigitalInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

const std::string& SetDigitalInstruction::getDescription() const { return description_; }

void SetDigitalInstruction::setDescription(const std::string& description) { description_ = description; }

void SetDigitalInstruction::print(const std::string& prefix) const  // NOLINT
{
  std::cout << prefix + "Set Digital Instruction, Key: " << key_ << ", Index: " << index_ << ", Value: " << value_;
  std::cout << ", Description: " << getDescription() << "\n";
}

std::unique_ptr<InstructionInterface> SetDigitalInstruction::clone() const
{
  return std::make_unique<SetDigitalInstruction>(*this);
}

std::string SetDigitalInstruction::getKey() const { return key_; }

int SetDigitalInstruction::getIndex() const { return index_; }

bool SetDigitalInstruction::getValue() const { return value_; }

bool SetDigitalInstruction::equals(const InstructionInterface& other) const
{
  const auto* rhs = dynamic_cast<const SetDigitalInstruction*>(&other);
  if (rhs == nullptr)
    return false;

  bool equal = true;
  equal &= (key_ == rhs->key_);
  equal &= (index_ == rhs->index_);
  equal &= (value_ == rhs->value_);
  return equal;
}

}  // namespace tesseract::command_language

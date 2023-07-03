/**
 * @file set_analog_instruction.cpp
 * @brief Set Analog Instruction
 *
 * @author Levi Armstrong
 * @date March 23, 2021
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
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
SetAnalogInstruction::SetAnalogInstruction(std::string key, int index, double value)
  : uuid_(boost::uuids::random_generator()()), key_(std::move(key)), index_(index), value_(value)
{
}

const boost::uuids::uuid& SetAnalogInstruction::getUUID() const { return uuid_; }
void SetAnalogInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("SetAnalogInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void SetAnalogInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& SetAnalogInstruction::getParentUUID() const { return parent_uuid_; }
void SetAnalogInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

const std::string& SetAnalogInstruction::getDescription() const { return description_; }

void SetAnalogInstruction::setDescription(const std::string& description) { description_ = description; }

void SetAnalogInstruction::print(const std::string& prefix) const  // NOLINT
{
  std::cout << prefix + "Set Analog Instruction, Key: " << key_ << ", Index: " << index_ << ", Value: " << value_;
  std::cout << ", Description: " << getDescription() << std::endl;
}

std::string SetAnalogInstruction::getKey() const { return key_; }

int SetAnalogInstruction::getIndex() const { return index_; }

double SetAnalogInstruction::getValue() const { return value_; }

bool SetAnalogInstruction::operator==(const SetAnalogInstruction& rhs) const
{
  // Check if they are the same
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (key_ == rhs.key_);
  equal &= (index_ == rhs.index_);
  equal &= (tesseract_common::almostEqualRelativeAndAbs(value_, rhs.value_, max_diff));
  return equal;
}
// LCOV_EXCL_START
bool SetAnalogInstruction::operator!=(const SetAnalogInstruction& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void SetAnalogInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("key", key_);
  ar& boost::serialization::make_nvp("index", index_);
  ar& boost::serialization::make_nvp("value", value_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SetAnalogInstruction)
TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::SetAnalogInstruction);

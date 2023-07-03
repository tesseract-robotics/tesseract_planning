/**
 * @file timer_instruction.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date November 15, 2020
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
#include <iostream>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
TimerInstruction::TimerInstruction(TimerInstructionType type, double time, int io)
  : uuid_(boost::uuids::random_generator()()), timer_type_(type), timer_time_(time), timer_io_(io)
{
}

const boost::uuids::uuid& TimerInstruction::getUUID() const { return uuid_; }
void TimerInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("TimerInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void TimerInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& TimerInstruction::getParentUUID() const { return parent_uuid_; }
void TimerInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

const std::string& TimerInstruction::getDescription() const { return description_; }

void TimerInstruction::setDescription(const std::string& description) { description_ = description; }

void TimerInstruction::print(const std::string& prefix) const  // NOLINT
{
  std::cout << prefix + "Timer Instruction, Timer Type: " << static_cast<int>(timer_type_) << ", Time: " << timer_time_
            << ", IO: " << timer_io_;
  std::cout << ", Description: " << getDescription() << std::endl;
}

TimerInstructionType TimerInstruction::getTimerType() const { return timer_type_; }

void TimerInstruction::setTimerType(TimerInstructionType type) { timer_type_ = type; }

double TimerInstruction::getTimerTime() const { return timer_time_; }

void TimerInstruction::setTimerTime(double time) { timer_time_ = time; }

int TimerInstruction::getTimerIO() const { return timer_io_; }

void TimerInstruction::setTimerIO(int io) { timer_io_ = io; }

bool TimerInstruction::operator==(const TimerInstruction& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(timer_time_, rhs.timer_time_, max_diff);
  equal &= (timer_type_ == rhs.timer_type_);
  equal &= (timer_io_ == rhs.timer_io_);

  return equal;
}
// LCOV_EXCL_START
bool TimerInstruction::operator!=(const TimerInstruction& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TimerInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("timer_type", timer_type_);
  ar& boost::serialization::make_nvp("timer_time", timer_time_);
  ar& boost::serialization::make_nvp("timer_io", timer_io_);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimerInstruction)
TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::TimerInstruction);

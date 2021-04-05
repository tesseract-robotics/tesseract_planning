/**
 * @file plan_instruction.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
PlanInstruction::PlanInstruction(Waypoint waypoint,
                                 PlanInstructionType type,
                                 std::string profile,
                                 ManipulatorInfo manipulator_info)
  : plan_type_(type), waypoint_(std::move(waypoint)), profile_(std::move(profile)), manipulator_info_(manipulator_info)
{
}

void PlanInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
Waypoint& PlanInstruction::getWaypoint() { return waypoint_; }
const Waypoint& PlanInstruction::getWaypoint() const { return waypoint_; }

void PlanInstruction::setManipulatorInfo(ManipulatorInfo info) { manipulator_info_ = info; }
const ManipulatorInfo& PlanInstruction::getManipulatorInfo() const { return manipulator_info_; }
ManipulatorInfo& PlanInstruction::getManipulatorInfo() { return manipulator_info_; }

void PlanInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? DEFAULT_PROFILE_KEY : profile;
}
const std::string& PlanInstruction::getProfile() const { return profile_; }

const std::string& PlanInstruction::getDescription() const { return description_; }

void PlanInstruction::setDescription(const std::string& description) { description_ = description; }

void PlanInstruction::print(const std::string& prefix) const
{
  std::cout << prefix + "Plan Instruction, Plan Type: " << static_cast<int>(plan_type_) << ", ";
  getWaypoint().print();
  std::cout << ", Description: " << getDescription() << std::endl;
}

PlanInstructionType PlanInstruction::getPlanType() const { return plan_type_; }

void PlanInstruction::setPlanType(PlanInstructionType type) { plan_type_ = type; }

bool PlanInstruction::isLinear() const { return (plan_type_ == PlanInstructionType::LINEAR); }

bool PlanInstruction::isFreespace() const { return (plan_type_ == PlanInstructionType::FREESPACE); }

bool PlanInstruction::isCircular() const { return (plan_type_ == PlanInstructionType::CIRCULAR); }

bool PlanInstruction::isStart() const { return (plan_type_ == PlanInstructionType::START); }

bool PlanInstruction::operator==(const PlanInstruction& rhs) const
{
  bool equal = true;
  equal &= (static_cast<int>(plan_type_) == static_cast<int>(rhs.plan_type_));
  equal &= (waypoint_ == rhs.waypoint_);
  equal &= (manipulator_info_ == rhs.manipulator_info_);
  equal &= (profile_ == rhs.profile_);  // NO LINT
  return equal;
}

bool PlanInstruction::operator!=(const PlanInstruction& rhs) const { return !operator==(rhs); }

template <class Archive>
void PlanInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("plan_type", plan_type_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("profile", profile_);
  ar& boost::serialization::make_nvp("waypoint", waypoint_);
  ar& boost::serialization::make_nvp("manipulator_info", manipulator_info_);
}

}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::PlanInstruction::serialize(boost::archive::xml_oarchive& ar,
                                                             const unsigned int version);
template void tesseract_planning::PlanInstruction::serialize(boost::archive::xml_iarchive& ar,
                                                             const unsigned int version);

TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::PlanInstruction);

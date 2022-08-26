/**
 * @file composite_instruction.cpp
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
#include <stdexcept>
#include <iostream>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_command_language/move_instruction.h> /** @todo Remove after refactor is complete */
namespace tesseract_planning
{
bool moveFilter(const InstructionPoly& instruction,
                const CompositeInstruction& /*composite*/,
                bool parent_is_first_composite)
{
  if (instruction.isMoveInstruction())
  {
    if (instruction.as<MoveInstructionPoly>().isStart())
      return (parent_is_first_composite);

    return true;
  }
  return false;
}

CompositeInstruction::CompositeInstruction(std::string profile,
                                           CompositeInstructionOrder order,
                                           tesseract_common::ManipulatorInfo manipulator_info)
  : manipulator_info_(std::move(manipulator_info)), profile_(std::move(profile)), order_(order)
{
}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

void CompositeInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? DEFAULT_PROFILE_KEY : profile;
}
const std::string& CompositeInstruction::getProfile() const { return profile_; }

void CompositeInstruction::setProfileOverrides(ProfileDictionary::ConstPtr profile_overrides)
{
  profile_overrides_ = std::move(profile_overrides);
}
ProfileDictionary::ConstPtr CompositeInstruction::getProfileOverrides() const { return profile_overrides_; }

void CompositeInstruction::setManipulatorInfo(tesseract_common::ManipulatorInfo info)
{
  manipulator_info_ = std::move(info);
}
const tesseract_common::ManipulatorInfo& CompositeInstruction::getManipulatorInfo() const { return manipulator_info_; }
tesseract_common::ManipulatorInfo& CompositeInstruction::getManipulatorInfo() { return manipulator_info_; }

void CompositeInstruction::setStartInstruction(MoveInstructionPoly instruction)
{
  start_instruction_ = std::move(instruction);
}

void CompositeInstruction::resetStartInstruction() { start_instruction_ = InstructionPoly(); }

const MoveInstructionPoly& CompositeInstruction::getStartInstruction() const
{
  return start_instruction_.as<MoveInstructionPoly>();
}

MoveInstructionPoly& CompositeInstruction::getStartInstruction()
{
  return start_instruction_.as<MoveInstructionPoly>();
}

bool CompositeInstruction::hasStartInstruction() const { return (!start_instruction_.isNull()); }

void CompositeInstruction::setInstructions(std::vector<InstructionPoly> instructions) { container_.swap(instructions); }

std::vector<InstructionPoly>& CompositeInstruction::getInstructions() { return container_; }

const std::vector<InstructionPoly>& CompositeInstruction::getInstructions() const { return container_; }

void CompositeInstruction::appendMoveInstruction(const MoveInstructionPoly& mi) { container_.emplace_back(mi); }

void CompositeInstruction::appendMoveInstruction(const MoveInstructionPoly&& mi) { container_.emplace_back(mi); }

void CompositeInstruction::appendInstruction(const InstructionPoly& i) { container_.push_back(i); }

void CompositeInstruction::appendInstruction(const InstructionPoly&& i) { container_.push_back(i); }

MoveInstructionPoly* CompositeInstruction::getFirstMoveInstruction()
{
  InstructionPoly* mi = getFirstInstruction(moveFilter);
  if (mi != nullptr)
    return &mi->as<MoveInstructionPoly>();

  return nullptr;
}

const MoveInstructionPoly* CompositeInstruction::getFirstMoveInstruction() const
{
  const InstructionPoly* mi = getFirstInstruction(moveFilter);
  if (mi != nullptr)
    return &mi->as<MoveInstructionPoly>();

  return nullptr;
}

MoveInstructionPoly* CompositeInstruction::getLastMoveInstruction()
{
  InstructionPoly* mi = getLastInstruction(moveFilter);
  if (mi != nullptr)
    return &mi->as<MoveInstructionPoly>();

  return nullptr;
}

const MoveInstructionPoly* CompositeInstruction::getLastMoveInstruction() const
{
  const InstructionPoly* mi = getLastInstruction(moveFilter);
  if (mi != nullptr)
    return &mi->as<MoveInstructionPoly>();

  return nullptr;
}

long CompositeInstruction::getMoveInstructionCount() const { return getInstructionCount(moveFilter); }

const InstructionPoly* CompositeInstruction::getFirstInstruction(const locateFilterFn& locate_filter,
                                                                 bool process_child_composites) const
{
  return getFirstInstructionHelper(*this, locate_filter, process_child_composites, true);
}

InstructionPoly* CompositeInstruction::getFirstInstruction(const locateFilterFn& locate_filter,
                                                           bool process_child_composites)
{
  return getFirstInstructionHelper(*this, locate_filter, process_child_composites, true);
}

const InstructionPoly* CompositeInstruction::getLastInstruction(const locateFilterFn& locate_filter,
                                                                bool process_child_composites) const
{
  return getLastInstructionHelper(*this, locate_filter, process_child_composites, true);
}

InstructionPoly* CompositeInstruction::getLastInstruction(const locateFilterFn& locate_filter,
                                                          bool process_child_composites)
{
  return getLastInstructionHelper(*this, locate_filter, process_child_composites, true);
}

long CompositeInstruction::getInstructionCount(const locateFilterFn& locate_filter, bool process_child_composites) const
{
  return getInstructionCountHelper(*this, locate_filter, process_child_composites, true);
}

std::vector<std::reference_wrapper<InstructionPoly>> CompositeInstruction::flatten(const flattenFilterFn& filter)
{
  std::vector<std::reference_wrapper<InstructionPoly>> flattened;
  flattenHelper(flattened, *this, filter, true);
  return flattened;
}

std::vector<std::reference_wrapper<const InstructionPoly>>
CompositeInstruction::flatten(const flattenFilterFn& filter) const
{
  std::vector<std::reference_wrapper<const InstructionPoly>> flattened;
  flattenHelper(flattened, *this, filter, true);
  return flattened;
}

std::vector<std::reference_wrapper<InstructionPoly>>
CompositeInstruction::flattenToPattern(const CompositeInstruction& pattern, const flattenFilterFn& filter)
{
  std::vector<std::reference_wrapper<InstructionPoly>> flattened;
  flattenToPatternHelper(flattened, *this, pattern, filter, true);
  return flattened;
}

std::vector<std::reference_wrapper<const InstructionPoly>>
CompositeInstruction::flattenToPattern(const CompositeInstruction& pattern, const flattenFilterFn& filter) const
{
  std::vector<std::reference_wrapper<const InstructionPoly>> flattened;
  flattenToPatternHelper(flattened, *this, pattern, filter, true);
  return flattened;
}

void CompositeInstruction::print(const std::string& prefix) const
{
  std::cout << prefix + "Composite Instruction, Description: " << getDescription() << std::endl;
  if (!start_instruction_.isNull())
    std::cout << prefix + "--- Start Instruction, Description: " << start_instruction_.getDescription() << std::endl;
  else
    std::cout << prefix + "--- Start Instruction, Description: Null Instruction" << std::endl;
  std::cout << prefix + "{" << std::endl;
  for (const auto& i : container_)
  {
    if (i.isNull())
      std::cout << prefix + "  Null Instruction" << std::endl;
    else
      i.print(prefix + "  ");
  }
  std::cout << prefix + "}" << std::endl;
}

bool CompositeInstruction::operator==(const CompositeInstruction& rhs) const
{
  bool equal = true;
  equal &= (static_cast<int>(order_) == static_cast<int>(rhs.order_));
  equal &= (profile_ == rhs.profile_);  // NOLINT
  equal &= (manipulator_info_ == rhs.manipulator_info_);
  equal &= (start_instruction_ == rhs.start_instruction_);
  equal &= (container_.size() == rhs.container_.size());
  if (equal)
  {
    for (std::size_t i = 0; i < container_.size(); ++i)
    {
      equal &= (container_[i] == rhs.container_[i]);

      if (!equal)
        break;
    }
  }
  return equal;
}

bool CompositeInstruction::operator!=(const CompositeInstruction& rhs) const { return !operator==(rhs); }

///////////////
// Iterators //
///////////////
CompositeInstruction::iterator CompositeInstruction::begin() { return container_.begin(); }
CompositeInstruction::const_iterator CompositeInstruction::begin() const { return container_.begin(); }
CompositeInstruction::iterator CompositeInstruction::end() { return container_.end(); }
CompositeInstruction::const_iterator CompositeInstruction::end() const { return container_.end(); }
CompositeInstruction::reverse_iterator CompositeInstruction::rbegin() { return container_.rbegin(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::rbegin() const { return container_.rbegin(); }
CompositeInstruction::reverse_iterator CompositeInstruction::rend() { return container_.rend(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::rend() const { return container_.rend(); }
CompositeInstruction::const_iterator CompositeInstruction::cbegin() const { return container_.cbegin(); }
CompositeInstruction::const_iterator CompositeInstruction::cend() const { return container_.cend(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::crbegin() const { return container_.crbegin(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::crend() const { return container_.crend(); }

//////////////
// Capacity //
//////////////
bool CompositeInstruction::empty() const { return container_.empty(); }
CompositeInstruction::size_type CompositeInstruction::size() const { return container_.size(); }
CompositeInstruction::size_type CompositeInstruction::max_size() const { return container_.max_size(); }
void CompositeInstruction::reserve(size_type n) { container_.reserve(n); }
CompositeInstruction::size_type CompositeInstruction::capacity() const { return container_.capacity(); }
void CompositeInstruction::shrink_to_fit() { container_.shrink_to_fit(); }

////////////////////
// Element Access //
////////////////////
CompositeInstruction::reference CompositeInstruction::front() { return container_.front(); }
CompositeInstruction::const_reference CompositeInstruction::front() const { return container_.front(); }
CompositeInstruction::reference CompositeInstruction::back() { return container_.back(); }
CompositeInstruction::const_reference CompositeInstruction::back() const { return container_.back(); }
CompositeInstruction::reference CompositeInstruction::at(size_type n) { return container_.at(n); }
CompositeInstruction::const_reference CompositeInstruction::at(size_type n) const { return container_.at(n); }
CompositeInstruction::pointer CompositeInstruction::data() { return container_.data(); }
CompositeInstruction::const_pointer CompositeInstruction::data() const { return container_.data(); }
CompositeInstruction::reference CompositeInstruction::operator[](size_type pos) { return container_[pos]; }
CompositeInstruction::const_reference CompositeInstruction::operator[](size_type pos) const { return container_[pos]; };

///////////////
// Modifiers //
///////////////
void CompositeInstruction::clear() { container_.clear(); }

CompositeInstruction::iterator CompositeInstruction::erase(const_iterator p) { return container_.erase(p); }
CompositeInstruction::iterator CompositeInstruction::erase(const_iterator first, const_iterator last)
{
  return container_.erase(first, last);
}

void CompositeInstruction::pop_back() { container_.pop_back(); }
void CompositeInstruction::swap(std::vector<value_type>& other) { container_.swap(other); }

///////////////////////////////////
/// Helper functions
//////////////////////////////////

const InstructionPoly*
CompositeInstruction::getFirstInstructionHelper(const CompositeInstruction& composite_instruction,
                                                const locateFilterFn& locate_filter,
                                                bool process_child_composites,
                                                bool first_composite) const
{
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
      return &(composite_instruction.start_instruction_);

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction.container_)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        return &instruction;

      if (instruction.isCompositeInstruction())
      {
        const InstructionPoly* result = getFirstInstructionHelper(
            instruction.as<CompositeInstruction>(), locate_filter, process_child_composites, false);
        if (result != nullptr)
          return result;
      }
    }

    return nullptr;
  }

  for (const auto& instruction : composite_instruction.container_)
    if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
      return &instruction;

  return nullptr;
}

InstructionPoly* CompositeInstruction::getFirstInstructionHelper(CompositeInstruction& composite_instruction,
                                                                 const locateFilterFn& locate_filter,
                                                                 bool process_child_composites,
                                                                 bool first_composite)
{
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
      return &(composite_instruction.start_instruction_);

  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction.container_)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        return &instruction;

      if (instruction.isCompositeInstruction())
      {
        InstructionPoly* result = getFirstInstructionHelper(
            instruction.as<CompositeInstruction>(), locate_filter, process_child_composites, false);
        if (result != nullptr)
          return result;
      }
    }

    return nullptr;
  }

  for (auto& instruction : composite_instruction.container_)
    if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
      return &instruction;

  return nullptr;
}

const InstructionPoly* CompositeInstruction::getLastInstructionHelper(const CompositeInstruction& composite_instruction,
                                                                      const locateFilterFn& locate_filter,
                                                                      bool process_child_composites,
                                                                      bool first_composite) const
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.container_.rbegin(); it != composite_instruction.container_.rend(); ++it)
    {
      if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
        return &(*it);

      if (it->isCompositeInstruction())
      {
        const InstructionPoly* result =
            getLastInstructionHelper(it->as<CompositeInstruction>(), locate_filter, process_child_composites, false);
        if (result != nullptr)
          return result;
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (!locate_filter ||
          locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
        return &(composite_instruction.start_instruction_);

    return nullptr;
  }

  for (auto it = composite_instruction.container_.rbegin(); it != composite_instruction.container_.rend(); ++it)
    if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
      return &(*it);

  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
      return &(composite_instruction.start_instruction_);

  return nullptr;
}

InstructionPoly* CompositeInstruction::getLastInstructionHelper(CompositeInstruction& composite_instruction,
                                                                const locateFilterFn& locate_filter,
                                                                bool process_child_composites,
                                                                bool first_composite)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.container_.rbegin(); it != composite_instruction.container_.rend(); ++it)
    {
      if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
        return &(*it);

      if (it->isCompositeInstruction())
      {
        InstructionPoly* result =
            getLastInstructionHelper(it->as<CompositeInstruction>(), locate_filter, process_child_composites, false);
        if (result != nullptr)
          return result;
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (!locate_filter ||
          locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
        return &(composite_instruction.start_instruction_);

    return nullptr;
  }

  for (auto it = composite_instruction.container_.rbegin(); it != composite_instruction.container_.rend(); ++it)
    if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
      return &(*it);

  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
      return &(composite_instruction.start_instruction_);

  return nullptr;
}

long CompositeInstruction::getInstructionCountHelper(const CompositeInstruction& composite_instruction,
                                                     const locateFilterFn& locate_filter,
                                                     bool process_child_composites,
                                                     bool first_composite) const
{
  long cnt = 0;
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.start_instruction_, composite_instruction, first_composite))
      ++cnt;

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction.container_)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        ++cnt;

      if (instruction.isCompositeInstruction())
        cnt += getInstructionCountHelper(
            instruction.as<CompositeInstruction>(), locate_filter, process_child_composites, false);
    }
    return cnt;
  }

  cnt += std::count_if(
      composite_instruction.container_.begin(), composite_instruction.container_.end(), [=](const auto& i) {
        return (!locate_filter || locate_filter(i, composite_instruction, first_composite));
      });

  return cnt;
}

void CompositeInstruction::flattenHelper(std::vector<std::reference_wrapper<InstructionPoly>>& flattened,
                                         CompositeInstruction& composite,
                                         const flattenFilterFn& filter,
                                         bool first_composite)
{
  if (composite.hasStartInstruction())
    if (!filter || filter(composite.start_instruction_, composite, first_composite))
      flattened.emplace_back(composite.start_instruction_);

  for (auto& i : composite.container_)
  {
    if (i.isCompositeInstruction())
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(i, composite, first_composite))
          flattened.emplace_back(i);

      flattenHelper(flattened, i.as<CompositeInstruction>(), filter, false);
    }
    else if (!filter || (filter && filter(i, composite, first_composite)))
    {
      flattened.emplace_back(i);
    }
  }
}

void CompositeInstruction::flattenHelper(std::vector<std::reference_wrapper<const InstructionPoly>>& flattened,
                                         const CompositeInstruction& composite,
                                         const flattenFilterFn& filter,
                                         bool first_composite) const
{
  if (composite.hasStartInstruction())
    if (!filter || filter(composite.start_instruction_, composite, first_composite))
      flattened.emplace_back(composite.start_instruction_);

  for (const auto& i : composite.container_)
  {
    if (i.isCompositeInstruction())
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(i, composite, first_composite))
          flattened.emplace_back(i);

      flattenHelper(flattened, i.as<CompositeInstruction>(), filter, false);
    }
    else if (!filter || filter(i, composite, first_composite))
    {
      flattened.emplace_back(i);
    }
  }
}

void CompositeInstruction::flattenToPatternHelper(std::vector<std::reference_wrapper<InstructionPoly>>& flattened,
                                                  CompositeInstruction& composite,
                                                  const CompositeInstruction& pattern,
                                                  const flattenFilterFn& filter,
                                                  bool first_composite)
{
  if (composite.container_.size() != pattern.container_.size() ||
      composite.hasStartInstruction() != pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  if (composite.hasStartInstruction())
    if (!filter || filter(composite.start_instruction_, composite, first_composite))
      flattened.emplace_back(composite.start_instruction_);

  for (std::size_t i = 0; i < pattern.container_.size(); i++)
  {
    if (pattern.container_.at(i).isCompositeInstruction() && composite.container_[i].isCompositeInstruction())
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(composite.container_[i], composite, first_composite))
          flattened.emplace_back(composite.container_[i]);

      flattenToPatternHelper(flattened,
                             composite.container_[i].as<CompositeInstruction>(),
                             pattern.container_.at(i).as<CompositeInstruction>(),
                             filter,
                             false);
    }
    else
    {
      flattened.emplace_back(composite.container_[i]);
    }
  }
}

void CompositeInstruction::flattenToPatternHelper(std::vector<std::reference_wrapper<const InstructionPoly>>& flattened,
                                                  const CompositeInstruction& composite,
                                                  const CompositeInstruction& pattern,
                                                  const flattenFilterFn& filter,
                                                  bool first_composite) const
{
  if (composite.container_.size() != pattern.container_.size() ||
      composite.hasStartInstruction() != pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  if (composite.hasStartInstruction())
    if (!filter || filter(composite.start_instruction_, composite, first_composite))
      flattened.emplace_back(composite.start_instruction_);

  for (std::size_t i = 0; i < pattern.container_.size(); i++)
  {
    if (pattern.container_.at(i).isCompositeInstruction() && composite.container_[i].isCompositeInstruction())
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(composite.container_[i], composite, first_composite))
          flattened.emplace_back(composite.container_[i]);

      flattenToPatternHelper(flattened,
                             composite.container_[i].as<CompositeInstruction>(),
                             pattern.container_.at(i).as<CompositeInstruction>(),
                             filter,
                             false);
    }
    else
    {
      flattened.emplace_back(composite.container_[i]);
    }
  }
}

template <class Archive>
void CompositeInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("manipulator_info", manipulator_info_);
  ar& boost::serialization::make_nvp("profile", profile_);
  ar& boost::serialization::make_nvp("order", order_);
  ar& boost::serialization::make_nvp("start_instruction", start_instruction_);
  ar& boost::serialization::make_nvp("container", container_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CompositeInstruction)
TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::CompositeInstruction);

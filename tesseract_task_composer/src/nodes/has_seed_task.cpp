/**
 * @file has_seed_task.cpp
 * @brief Task for checking if seed exists
 *
 * @author Levi Armstrong
 * @date November 2. 2021
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
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/has_seed_task.h>
#include <tesseract_command_language/composite_instruction.h>
//#include <tesseract_process_managers/core/utils.h>

namespace tesseract_planning
{
HasSeedTask::HasSeedTask(std::string input_key, std::string name)
  : TaskComposerNode(std::move(name)), input_key_(std::move(input_key))
{
}

bool isCompositeEmpty(const CompositeInstruction& composite)
{
  if (composite.empty())
    return true;

  for (const auto& i : composite)
  {
    if (i.isCompositeInstruction())
    {
      const auto& sub_composite = i.as<CompositeInstruction>();
      if (isCompositeEmpty(sub_composite))
        return true;
    }
  }

  return false;
}

int HasSeedTask::run(TaskComposerInput& input) const
{
  auto seed_data_poly = input.data_storage->getData(input_key_);
  if (seed_data_poly.isNull() || seed_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
    return 0;

  const auto& composite = seed_data_poly.as<CompositeInstruction>();
  if (isCompositeEmpty(composite))
  {
    CONSOLE_BRIDGE_logDebug("Seed is empty!");
    return 0;
  }

  return 1;
}

bool HasSeedTask::operator==(const HasSeedTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool HasSeedTask::operator!=(const HasSeedTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void HasSeedTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

HasSeedTaskInfo::HasSeedTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr HasSeedTaskInfo::clone() const { return std::make_unique<HasSeedTaskInfo>(*this); }

bool HasSeedTaskInfo::operator==(const HasSeedTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool HasSeedTaskInfo::operator!=(const HasSeedTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void HasSeedTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::HasSeedTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::HasSeedTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::HasSeedTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::HasSeedTaskInfo)

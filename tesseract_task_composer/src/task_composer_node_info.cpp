/**
 * @file task_composer_node_info.cpp
 * @brief Task composer node info
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
TaskComposerNodeInfo::TaskComposerNodeInfo(boost::uuids::uuid uuid, std::string name)
  : name(std::move(name)), uuid(uuid)
{
}

bool TaskComposerNodeInfo::operator==(const TaskComposerNodeInfo& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= name == rhs.name;
  equal &= uuid == rhs.uuid;
  equal &= results == rhs.results;
  equal &= return_value == rhs.return_value;
  equal &= message == rhs.message;
  equal &= tesseract_common::almostEqualRelativeAndAbs(elapsed_time, rhs.elapsed_time, max_diff);
  equal &= tesseract_common::isIdentical(input_keys, rhs.input_keys, false);
  equal &= tesseract_common::isIdentical(output_keys, rhs.output_keys, false);
  return equal;
}

bool TaskComposerNodeInfo::operator!=(const TaskComposerNodeInfo& rhs) const { return !operator==(rhs); }

TaskComposerNodeInfo::UPtr TaskComposerNodeInfo::clone() const { return std::make_unique<TaskComposerNodeInfo>(*this); }

template <class Archive>
void TaskComposerNodeInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name);
  ar& boost::serialization::make_nvp("uuid", uuid);
  ar& boost::serialization::make_nvp("results", results);
  ar& boost::serialization::make_nvp("return_value", return_value);
  ar& boost::serialization::make_nvp("message", message);
  ar& boost::serialization::make_nvp("elapsed_time", elapsed_time);

  ar& boost::serialization::make_nvp("input_keys", input_keys);
  ar& boost::serialization::make_nvp("output_keys", output_keys);
}

TaskComposerNodeInfoContainer::TaskComposerNodeInfoContainer(const TaskComposerNodeInfoContainer& other)
{
  *this = other;
}
TaskComposerNodeInfoContainer& TaskComposerNodeInfoContainer::operator=(const TaskComposerNodeInfoContainer& other)
{
  std::shared_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  for (const auto& pair : other.info_map_)
    info_map_[pair.first] = pair.second->clone();

  return *this;
}

void TaskComposerNodeInfoContainer::addInfo(TaskComposerNodeInfo::UPtr info)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  info_map_[info->uuid] = std::move(info);
}

const TaskComposerNodeInfo& TaskComposerNodeInfoContainer::getInfo(boost::uuids::uuid key) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return *info_map_.at(key);
}

void TaskComposerNodeInfoContainer::clear()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return info_map_.clear();
}

const TaskComposerNodeInfo& TaskComposerNodeInfoContainer::operator[](boost::uuids::uuid key) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return *info_map_.at(key);
}

std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> TaskComposerNodeInfoContainer::getInfoMap() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> copy;
  for (const auto& pair : info_map_)
    copy[pair.first] = pair.second->clone();
  return copy;
}

bool TaskComposerNodeInfoContainer::operator==(const TaskComposerNodeInfoContainer& rhs) const
{
  bool equal = true;
  auto equality = [](const TaskComposerNodeInfo::UPtr& p1, const TaskComposerNodeInfo::UPtr& p2) {
    return (p1 && p2 && *p1 == *p2) || (!p1 && !p2);
  };
  equal &= tesseract_common::isIdenticalMap<std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>,
                                            TaskComposerNodeInfo::UPtr>(info_map_, rhs.info_map_, equality);
  return equal;
}

bool TaskComposerNodeInfoContainer::operator!=(const TaskComposerNodeInfoContainer& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void TaskComposerNodeInfoContainer::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(info_map_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNodeInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNodeInfo)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNodeInfoContainer)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNodeInfoContainer)

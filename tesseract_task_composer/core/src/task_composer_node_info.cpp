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
#include <boost/serialization/binary_object.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
#include <mutex>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_graph.h>

namespace tesseract_planning
{
TaskComposerNodeInfo::TaskComposerNodeInfo(const TaskComposerNode& node)
  : name(node.name_)
  , ns(node.ns_)
  , uuid(node.uuid_)
  , parent_uuid(node.parent_uuid_)
  , type(node.type_)
  , type_hash_code(std::type_index(typeid(node)).hash_code())
  , conditional(node.conditional_)
  , inbound_edges(node.inbound_edges_)
  , outbound_edges(node.outbound_edges_)
  , input_keys(node.input_keys_)
  , output_keys(node.output_keys_)
  , triggers_abort(node.trigger_abort_)
{
  if (type == TaskComposerNodeType::GRAPH || type == TaskComposerNodeType::PIPELINE)
  {
    const auto& graph = static_cast<const TaskComposerGraph&>(node);
    root_uuid = graph.getRootNode();
    terminals = graph.getTerminals();
    triggers_abort = (graph.getAbortTerminalIndex() >= 0);
  }
}

TaskComposerNodeInfo::~TaskComposerNodeInfo() = default;

bool TaskComposerNodeInfo::operator==(const TaskComposerNodeInfo& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= name == rhs.name;
  equal &= ns == rhs.ns;
  equal &= uuid == rhs.uuid;
  equal &= root_uuid == rhs.root_uuid;
  equal &= parent_uuid == rhs.parent_uuid;
  equal &= type == rhs.type;
  equal &= type_hash_code == rhs.type_hash_code;
  equal &= conditional == rhs.conditional;
  equal &= return_value == rhs.return_value;
  equal &= status_code == rhs.status_code;
  equal &= status_message == rhs.status_message;
  equal &= start_time == rhs.start_time;
  equal &= tesseract_common::almostEqualRelativeAndAbs(elapsed_time, rhs.elapsed_time, max_diff);
  equal &= tesseract_common::isIdentical(inbound_edges, rhs.inbound_edges, false);
  equal &= tesseract_common::isIdentical(outbound_edges, rhs.outbound_edges, true);
  equal &= input_keys == rhs.input_keys;
  equal &= output_keys == rhs.output_keys;
  equal &= terminals == rhs.terminals;
  equal &= triggers_abort == rhs.triggers_abort;
  equal &= color == rhs.color;
  equal &= dotgraph == rhs.dotgraph;
  equal &= data_storage == rhs.data_storage;
  equal &= aborted == rhs.aborted;
  return equal;
}

bool TaskComposerNodeInfo::operator!=(const TaskComposerNodeInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerNodeInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name);
  ar& boost::serialization::make_nvp("ns", ns);
  ar& boost::serialization::make_nvp("uuid", uuid);
  ar& boost::serialization::make_nvp("root_uuid", root_uuid);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid);
  ar& boost::serialization::make_nvp("type", type);
  ar& boost::serialization::make_nvp("type_hash_code", type_hash_code);
  ar& boost::serialization::make_nvp("conditional", conditional);
  ar& boost::serialization::make_nvp("return_value", return_value);
  ar& boost::serialization::make_nvp("status_code", status_code);
  ar& boost::serialization::make_nvp("status_message", status_message);
  ar& boost::serialization::make_nvp("start_time",
                                     boost::serialization::make_binary_object(&start_time, sizeof(start_time)));
  ar& boost::serialization::make_nvp("elapsed_time", elapsed_time);
  ar& boost::serialization::make_nvp("inbound_edges", inbound_edges);
  ar& boost::serialization::make_nvp("outbound_edges", outbound_edges);
  ar& boost::serialization::make_nvp("input_keys", input_keys);
  ar& boost::serialization::make_nvp("output_keys", output_keys);
  ar& boost::serialization::make_nvp("terminals", terminals);
  ar& boost::serialization::make_nvp("triggers_abort", triggers_abort);
  ar& boost::serialization::make_nvp("color", color);
  ar& boost::serialization::make_nvp("dotgraph", dotgraph);
  ar& boost::serialization::make_nvp("data_storage", data_storage);
  ar& boost::serialization::make_nvp("aborted", aborted);
}

TaskComposerNodeInfoContainer::TaskComposerNodeInfoContainer(const TaskComposerNodeInfoContainer& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  aborting_node_ = other.aborting_node_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  info_map_ = other.info_map_;            // NOLINT(cppcoreguidelines-prefer-member-initializer)
}
TaskComposerNodeInfoContainer& TaskComposerNodeInfoContainer::operator=(const TaskComposerNodeInfoContainer& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  aborting_node_ = other.aborting_node_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  info_map_ = other.info_map_;            // NOLINT(cppcoreguidelines-prefer-member-initializer)

  return *this;
}

TaskComposerNodeInfoContainer::TaskComposerNodeInfoContainer(TaskComposerNodeInfoContainer&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  aborting_node_ = other.aborting_node_;   // NOLINT(cppcoreguidelines-prefer-member-initializer)
  info_map_ = std::move(other.info_map_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}
TaskComposerNodeInfoContainer& TaskComposerNodeInfoContainer::operator=(TaskComposerNodeInfoContainer&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  aborting_node_ = other.aborting_node_;   // NOLINT(cppcoreguidelines-prefer-member-initializer)
  info_map_ = std::move(other.info_map_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)

  return *this;
}

void TaskComposerNodeInfoContainer::addInfo(const TaskComposerNodeInfo& info)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  info_map_[info.uuid] = info;
}

std::optional<TaskComposerNodeInfo> TaskComposerNodeInfoContainer::getInfo(const boost::uuids::uuid& key) const
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = info_map_.find(key);
  if (it == info_map_.end())
    return std::nullopt;

  return it->second;
}

std::vector<TaskComposerNodeInfo>
TaskComposerNodeInfoContainer::find(const std::function<bool(const TaskComposerNodeInfo&)>& search_fn) const
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  std::vector<TaskComposerNodeInfo> results;
  for (const auto& info : info_map_)
  {
    if (search_fn(info.second))
      results.push_back(info.second);
  }
  return results;
}

void TaskComposerNodeInfoContainer::setRootNode(const boost::uuids::uuid& node_uuid)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  root_node_ = node_uuid;
}

boost::uuids::uuid TaskComposerNodeInfoContainer::getRootNode() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return root_node_;
}

void TaskComposerNodeInfoContainer::setAborted(const boost::uuids::uuid& node_uuid)
{
  assert(!node_uuid.is_nil());
  std::unique_lock<std::shared_mutex> lock(mutex_);
  aborting_node_ = node_uuid;
}

boost::uuids::uuid TaskComposerNodeInfoContainer::getAbortingNode() const
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return aborting_node_;
}

void TaskComposerNodeInfoContainer::clear()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  aborting_node_ = boost::uuids::uuid{};
  info_map_.clear();
}

void TaskComposerNodeInfoContainer::prune(const std::function<void(TaskComposerNodeInfo& node_info)>& prune_fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  for (auto& info : info_map_)
    prune_fn(info.second);
}

std::map<boost::uuids::uuid, TaskComposerNodeInfo> TaskComposerNodeInfoContainer::getInfoMap() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::map<boost::uuids::uuid, TaskComposerNodeInfo> copy;
  for (const auto& pair : info_map_)
    copy[pair.first] = pair.second;

  if (!aborting_node_.is_nil())
    updateParents(copy, aborting_node_);

  return copy;
}

void TaskComposerNodeInfoContainer::insertInfoMap(const TaskComposerNodeInfoContainer& container)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(container.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };
  for (const auto& pair : container.info_map_)
    info_map_[pair.first] = pair.second;
}

// NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
void TaskComposerNodeInfoContainer::mergeInfoMap(TaskComposerNodeInfoContainer&& container)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(container.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };
  info_map_.merge(std::move(container.info_map_));

  // Should be empty, but if not then duplicates keys exist which should not be possible.
  assert(container.info_map_.empty());
}

void TaskComposerNodeInfoContainer::updateParents(std::map<boost::uuids::uuid, TaskComposerNodeInfo>& info_map,
                                                  const boost::uuids::uuid& uuid) const
{
  auto it = info_map.find(uuid);
  if (it == info_map.end())
    return;

  if (it->second.parent_uuid.is_nil())
    return;

  auto parent_it = info_map.find(it->second.parent_uuid);
  if (parent_it == info_map.end())
    return;

  parent_it->second.color = it->second.color;

  updateParents(info_map, it->second.parent_uuid);
}

bool TaskComposerNodeInfoContainer::operator==(const TaskComposerNodeInfoContainer& rhs) const
{
  std::shared_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(rhs.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  bool equal = true;
  equal &= root_node_ == rhs.root_node_;
  equal &= aborting_node_ == rhs.aborting_node_;
  equal &= tesseract_common::isIdenticalMap<std::map<boost::uuids::uuid, TaskComposerNodeInfo>, TaskComposerNodeInfo>(
      info_map_, rhs.info_map_);
  return equal;
}

bool TaskComposerNodeInfoContainer::operator!=(const TaskComposerNodeInfoContainer& rhs) const
{
  std::shared_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(rhs.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  return !operator==(rhs);
}

template <class Archive>
void TaskComposerNodeInfoContainer::serialize(Archive& ar, const unsigned int /*version*/)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  ar& BOOST_SERIALIZATION_NVP(root_node_);
  ar& BOOST_SERIALIZATION_NVP(aborting_node_);
  ar& BOOST_SERIALIZATION_NVP(info_map_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNodeInfo)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNodeInfoContainer)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNodeInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNodeInfoContainer)

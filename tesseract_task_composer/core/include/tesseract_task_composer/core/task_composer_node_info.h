/**
 * @file task_composer_node_info.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <shared_mutex>
#include <map>
#include <chrono>
#include <functional>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/any_poly.h>

#include <tesseract_task_composer/core/task_composer_node_types.h>
#include <tesseract_task_composer/core/task_composer_keys.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

namespace tesseract_planning
{
class TaskComposerNode;

/** Stores information about a node */
class TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<TaskComposerNodeInfo>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeInfo>;
  using UPtr = std::unique_ptr<TaskComposerNodeInfo>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNodeInfo>;

  TaskComposerNodeInfo() = default;  // Required for serialization
  TaskComposerNodeInfo(const TaskComposerNode& node);
  ~TaskComposerNodeInfo();
  TaskComposerNodeInfo(const TaskComposerNodeInfo&) = default;
  TaskComposerNodeInfo& operator=(const TaskComposerNodeInfo&) = default;
  TaskComposerNodeInfo(TaskComposerNodeInfo&&) = default;
  TaskComposerNodeInfo& operator=(TaskComposerNodeInfo&&) = default;

  /** @brief The name of the task */
  std::string name;

  /** @brief The namespace of the task */
  std::string ns;

  /** @brief The task uuid */
  boost::uuids::uuid uuid{};

  /** @brief If type is Pipeline or Graph this will be the root node of the pipeline or graph, otherwise null */
  boost::uuids::uuid root_uuid{};

  /**
   * @brief The parent uuid
   * @details This is set when the node is added to a graph
   */
  boost::uuids::uuid parent_uuid{};

  /** @brief The node type */
  TaskComposerNodeType type{ TaskComposerNodeType::NODE };

  /** @brief The task type hash code from std::type_index */
  std::size_t type_hash_code{ 0 };

  /** @brief The task is conditional or not */
  bool conditional{ false };

  /** @brief The nodes inbound edges */
  std::vector<boost::uuids::uuid> inbound_edges;

  /** @brief The nodes outbound edges */
  std::vector<boost::uuids::uuid> outbound_edges;

  /** @brief The input keys */
  TaskComposerKeys input_keys;

  /** @brief The output keys */
  TaskComposerKeys output_keys;

  /** @brief The graph of pipeline terminals */
  std::vector<boost::uuids::uuid> terminals;

  /** @brief Indicate if it can trigger a abort. */
  bool triggers_abort{ false };

  /** @brief Value returned from the Task on completion */
  int return_value{ -1 };

  /** @brief Status code */
  int status_code{ 0 };

  /** @brief Status message */
  std::string status_message;

  /** @brief The start time */
  std::chrono::system_clock::time_point start_time{ std::chrono::system_clock::now() };

  /**
   * @brief Time spent in this task in seconds
   * @details This is managed by core components so implementation do not need to calculate this
   */
  double elapsed_time{ 0 };

  /** @brief The DOT Graph color to fill with */
  std::string color{ "red" };

  /**
   *  @brief dot graph string for visualization
   *  @brief This should only be populated if node generates dynamic nodes
   */
  std::string dotgraph;

  /** @brief This provides a location for task data may be stored */
  TaskComposerDataStorage data_storage;

  bool operator==(const TaskComposerNodeInfo& rhs) const;
  bool operator!=(const TaskComposerNodeInfo& rhs) const;

  /**
   * @brief Check if task was not ran because process was aborted
   * @return True if aborted otherwise false;
   */
  bool isAborted() const;

private:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  friend class TaskComposerNode;

  /** @brief Indicate if task was not ran because abort flag was enabled */
  bool aborted_{ false };

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief A threadsafe container for TaskComposerNodeInfo */
class TaskComposerNodeInfoContainer
{
public:
  using Ptr = std::shared_ptr<TaskComposerNodeInfoContainer>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeInfoContainer>;
  using UPtr = std::unique_ptr<TaskComposerNodeInfoContainer>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNodeInfoContainer>;

  TaskComposerNodeInfoContainer() = default;
  ~TaskComposerNodeInfoContainer() = default;
  TaskComposerNodeInfoContainer(const TaskComposerNodeInfoContainer&);
  TaskComposerNodeInfoContainer& operator=(const TaskComposerNodeInfoContainer&);
  TaskComposerNodeInfoContainer(TaskComposerNodeInfoContainer&&) noexcept;
  TaskComposerNodeInfoContainer& operator=(TaskComposerNodeInfoContainer&&) noexcept;

  /**
   * @brief Add info to the container
   * @param info The info to be added
   */
  void addInfo(TaskComposerNodeInfo::UPtr info);

  /**
   * @brief Get info for the provided key
   * @param key The key to retrieve info for
   * @return If key does not exist nullptr, otherwise a clone of the info
   */
  TaskComposerNodeInfo::UPtr getInfo(const boost::uuids::uuid& key) const;

  /**
   * @brief Get task info by name
   * @param name The name of task info to search
   * @param ns The namespace to search under. If empty all namespace's are include
   * @return A list of task infos with the following name.
   */
  std::vector<TaskComposerNodeInfo::UPtr>
  find(const std::function<bool(const TaskComposerNodeInfo& node_info)>& search_fn) const;

  /** @brief Get a copy of the task_info_map_ in case it gets resized*/
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> getInfoMap() const;

  /** @brief Insert the contents of another container's info map */
  void insertInfoMap(const TaskComposerNodeInfoContainer& container);

  /** @brief Merge the contents of another container's info map */
  void mergeInfoMap(TaskComposerNodeInfoContainer&& container);

  /** @brief Set the root node */
  void setRootNode(const boost::uuids::uuid& node_uuid);

  /** @brief Get the root node */
  boost::uuids::uuid getRootNode() const;

  /**
   * @brief Called if aborted
   * @details This is set if abort is called in input
   */
  void setAborted(const boost::uuids::uuid& node_uuid);

  /**
   * @brief Get the aborting node
   * @return Null if not set, otherwise nodes uuid
   */
  boost::uuids::uuid getAbortingNode() const;

  /** @brief Clear the contents */
  void clear();

  /** @brief Prune data from node infos to save space */
  void prune(const std::function<void(TaskComposerNodeInfo&)>& prune_fn);

  bool operator==(const TaskComposerNodeInfoContainer& rhs) const;
  bool operator!=(const TaskComposerNodeInfoContainer& rhs) const;

private:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  mutable std::shared_mutex mutex_;
  boost::uuids::uuid root_node_{};
  boost::uuids::uuid aborting_node_{};
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> info_map_;

  void updateParents(std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& info_map,
                     const boost::uuids::uuid& uuid) const;
};
}  // namespace tesseract_planning
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerNodeInfo)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerNodeInfoContainer)
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H

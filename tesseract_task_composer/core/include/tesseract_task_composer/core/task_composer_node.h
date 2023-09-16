/**
 * @file task_composer_node.h
 * @brief A node in the pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

namespace tesseract_planning
{
enum class TaskComposerNodeType
{
  NODE,
  TASK,
  PIPELINE,
  GRAPH
};

/** @brief Represents a node the pipeline to be executed */
class TaskComposerNode
{
public:
  using Ptr = std::shared_ptr<TaskComposerNode>;
  using ConstPtr = std::shared_ptr<const TaskComposerNode>;
  using UPtr = std::unique_ptr<TaskComposerNode>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNode>;

  TaskComposerNode(std::string name = "TaskComposerNode",
                   TaskComposerNodeType type = TaskComposerNodeType::NODE,
                   bool conditional = false);
  explicit TaskComposerNode(std::string name, TaskComposerNodeType type, const YAML::Node& config);
  virtual ~TaskComposerNode() = default;
  TaskComposerNode(const TaskComposerNode&) = delete;
  TaskComposerNode& operator=(const TaskComposerNode&) = delete;
  TaskComposerNode(TaskComposerNode&&) = delete;
  TaskComposerNode& operator=(TaskComposerNode&&) = delete;

  /** @brief Set the name of the node */
  void setName(const std::string& name);

  /** @brief The name of the node */
  const std::string& getName() const;

  /** @brief The node type TASK, GRAPH, etc */
  TaskComposerNodeType getType() const;

  /** @brief The node uuid */
  const boost::uuids::uuid& getUUID() const;

  /** @brief The node uuid */
  const std::string& getUUIDString() const;

  /**
   * @brief The parent uuid which can be null
   * @details This is not null if the node is part of a graph
   */
  const boost::uuids::uuid& getParentUUID() const;

  /**
   * @brief Check if node is conditional
   * @return
   */
  bool isConditional() const;

  /** @brief IDs of nodes (i.e. node) that should run after this node */
  const std::vector<boost::uuids::uuid>& getOutboundEdges() const;

  /** @brief IDs of nodes (i.e. node) that should run before this node */
  const std::vector<boost::uuids::uuid>& getInboundEdges() const;

  /** @brief Set the nodes input keys */
  void setInputKeys(const std::vector<std::string>& input_keys);

  /** @brief The nodes input keys */
  const std::vector<std::string>& getInputKeys() const;

  /** @brief Set the nodes input keys */
  void setOutputKeys(const std::vector<std::string>& output_keys);

  /** @brief The nodes output keys */
  const std::vector<std::string>& getOutputKeys() const;

  /** @brief Rename input keys */
  virtual void renameInputKeys(const std::map<std::string, std::string>& input_keys);

  /** @brief Rename output keys */
  virtual void renameOutputKeys(const std::map<std::string, std::string>& output_keys);

  /** @brief Set if conditional */
  virtual void setConditional(bool enable);

  /**
   * @brief dump the task to dot
   * @brief Return additional subgraphs which should get appended if needed
   */
  virtual std::string dump(std::ostream& os,
                           const TaskComposerNode* parent = nullptr,
                           const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map = {}) const;

  bool operator==(const TaskComposerNode& rhs) const;
  bool operator!=(const TaskComposerNode& rhs) const;

protected:
  friend class TaskComposerGraph;
  friend class TaskComposerNodeInfo;
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  /** @brief The name of the task */
  std::string name_;

  /** @brief The node type */
  TaskComposerNodeType type_;

  /** @brief The task uuid */
  boost::uuids::uuid uuid_;

  /** @brief The uuid as string */
  std::string uuid_str_;

  /**
   * @brief The parent uuid
   * @details This is set when the node is added to a graph
   */
  boost::uuids::uuid parent_uuid_{};

  /** @brief IDs of nodes (i.e. tasks) that should run after this node */
  std::vector<boost::uuids::uuid> outbound_edges_;

  /** @brief IDs of nodes (i.e. tasks) that should run before this node */
  std::vector<boost::uuids::uuid> inbound_edges_;

  /** @brief The nodes input keys */
  std::vector<std::string> input_keys_;

  /** @brief The nodes output keys */
  std::vector<std::string> output_keys_;

  /** @brief Indicate if node is conditional */
  bool conditional_{ false };

  /** @brief This will create a UUID string with no hyphens used when creating dot graph */
  static std::string toString(const boost::uuids::uuid& u, const std::string& prefix = "");
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerNode, "TaskComposerNode")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_H

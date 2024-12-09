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
#include <map>
#include <optional>
#include <boost/uuid/uuid.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>

#include <tesseract_task_composer/core/task_composer_keys.h>
#include <tesseract_task_composer/core/task_composer_node_types.h>
#include <tesseract_task_composer/core/task_composer_node_ports.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TaskComposerDataStorage;
class TaskComposerContext;
class TaskComposerExecutor;

/** @brief Represents a node the pipeline to be executed */
class TaskComposerNode
{
public:
  using Ptr = std::shared_ptr<TaskComposerNode>;
  using ConstPtr = std::shared_ptr<const TaskComposerNode>;
  using UPtr = std::unique_ptr<TaskComposerNode>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNode>;

  // @brief The results map
  using ResultsMap = std::map<boost::uuids::uuid, std::unique_ptr<TaskComposerNodeInfo>>;

  /** @brief Most task will not require a executor so making it optional */
  using OptionalTaskComposerExecutor = std::optional<std::reference_wrapper<TaskComposerExecutor>>;

  TaskComposerNode(std::string name = "TaskComposerNode",
                   TaskComposerNodeType type = TaskComposerNodeType::NODE,
                   TaskComposerNodePorts ports = TaskComposerNodePorts(),
                   bool conditional = false);
  explicit TaskComposerNode(std::string name,
                            TaskComposerNodeType type,
                            TaskComposerNodePorts ports,
                            const YAML::Node& config);
  virtual ~TaskComposerNode() = default;
  TaskComposerNode(const TaskComposerNode&) = delete;
  TaskComposerNode& operator=(const TaskComposerNode&) = delete;
  TaskComposerNode(TaskComposerNode&&) = delete;
  TaskComposerNode& operator=(TaskComposerNode&&) = delete;

  int run(TaskComposerContext& context, OptionalTaskComposerExecutor executor = std::nullopt) const;

  /** @brief Set the name of the node */
  void setName(const std::string& name);

  /** @brief The name of the node */
  const std::string& getName() const;

  /** @brief Set the namespace of the node */
  void setNamespace(const std::string& ns);

  /** @brief The namespace of the node */
  const std::string& getNamespace() const;

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

  /** @brief This will validate that all required ports exist and that no extra ports exist */
  void validatePorts() const;

  /** @brief IDs of nodes (i.e. node) that should run after this node */
  const std::vector<boost::uuids::uuid>& getOutboundEdges() const;

  /** @brief IDs of nodes (i.e. node) that should run before this node */
  const std::vector<boost::uuids::uuid>& getInboundEdges() const;

  /** @brief Set the nodes input keys */
  void setInputKeys(const TaskComposerKeys& input_keys);

  /** @brief The nodes input keys */
  const TaskComposerKeys& getInputKeys() const;

  /** @brief Set the nodes input keys */
  void setOutputKeys(const TaskComposerKeys& output_keys);

  /** @brief The nodes output keys */
  const TaskComposerKeys& getOutputKeys() const;

  /** @brief Get the ports associated with the node */
  TaskComposerNodePorts getPorts() const;

  /** @brief Generate the Dotgraph as a string */
  std::string getDotgraph(const ResultsMap& results_map = ResultsMap()) const;

  /** @brief Generate the Dotgraph and save to file */
  bool saveDotgraph(const std::string& filepath, const ResultsMap& results_map = ResultsMap()) const;  // NOLINT

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
                           const ResultsMap& results_map = ResultsMap()) const;

  bool operator==(const TaskComposerNode& rhs) const;
  bool operator!=(const TaskComposerNode& rhs) const;

protected:
  friend class TaskComposerGraph;
  friend class TaskComposerNodeInfo;
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  virtual std::unique_ptr<TaskComposerNodeInfo> runImpl(TaskComposerContext& context,
                                                        OptionalTaskComposerExecutor executor = std::nullopt) const = 0;

  /** @brief The name of the task */
  std::string name_;

  /** @brief The namespace of the task */
  std::string ns_;

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
  TaskComposerKeys input_keys_;

  /** @brief The nodes output keys */
  TaskComposerKeys output_keys_;

  /** @brief Indicate if node is conditional */
  bool conditional_{ false };

  /** @brief The nodes ports definition */
  TaskComposerNodePorts ports_;

  /** @brief Indicate if task triggers abort */
  bool trigger_abort_{ false };

  /** @brief This will create a UUID string with no hyphens used when creating dot graph */
  static std::string toString(const boost::uuids::uuid& u, const std::string& prefix = "");

  /**
   * @brief A utility function for extracting data from data storage
   * @param data_storage The data storage to retrieve data from
   * @param port The port associated with the key
   * @param required Indicate if data is required
   * @return The data stored under the name, if not found and required an exception will be thrown other null
   */
  template <typename T = tesseract_common::AnyPoly>
  T getData(const TaskComposerDataStorage& data_storage, const std::string& port, bool required = true) const;

  /**
   * @brief A utility function for setting data in data storage
   * @param port The port associated with the key
   * @param data_storage The data storage to assign data to
   * @param data The data to store
   * @param required Indicate if required port
   */
  void setData(TaskComposerDataStorage& data_storage,
               const std::string& port,
               tesseract_common::AnyPoly data,
               bool required = true) const;
  void setData(TaskComposerDataStorage& data_storage,
               const std::string& port,
               const std::vector<tesseract_common::AnyPoly>& data,
               bool required = true) const;
};

}  // namespace tesseract_planning
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerNode)
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_H

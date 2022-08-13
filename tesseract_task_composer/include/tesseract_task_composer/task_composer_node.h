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

#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_data_storage.h>

namespace tesseract_planning
{
class TaskComposerGraph;

enum class TaskComposerNodeType
{
  TASK,
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

  TaskComposerNode(std::string name = "TaskComposerNode", TaskComposerNodeType type = TaskComposerNodeType::TASK);
  virtual ~TaskComposerNode() = default;
  TaskComposerNode(const TaskComposerNode&) = delete;
  TaskComposerNode& operator=(const TaskComposerNode&) = delete;
  TaskComposerNode(TaskComposerNode&&) = delete;
  TaskComposerNode& operator=(TaskComposerNode&&) = delete;

  /** @brief The name of the node */
  const std::string& getName() const;

  /** @brief The node type TASK, GRAPH, etc */
  TaskComposerNodeType getType() const;

  /** @brief The task uuid */
  const boost::uuids::uuid& getUUID() const;

  /** @brief IDs of nodes (i.e. node) that should run after this node */
  const std::vector<boost::uuids::uuid>& getEdges() const;

  virtual int run(TaskComposerInput& input) const = 0;

  bool operator==(const TaskComposerNode& rhs) const;
  bool operator!=(const TaskComposerNode& rhs) const;

protected:
  friend class TaskComposerGraph;
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  /** @brief The name of the task */
  std::string name_;

  /** @brief The node type */
  TaskComposerNodeType type_;

  /** @brief The task uuid */
  boost::uuids::uuid uuid_;

  /** @brief IDs of nodes (i.e. tasks) that should run after this node */
  std::vector<boost::uuids::uuid> edges_;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerNode, "TaskComposerNode")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_H

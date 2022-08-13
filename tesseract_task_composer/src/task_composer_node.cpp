/**
 * @file task_composer_node.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <boost/serialization/vector.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node.h>

namespace tesseract_planning
{
TaskComposerNode::TaskComposerNode(std::string name, TaskComposerNodeType type)
  : name_(std::move(name)), type_(type), uuid_(boost::uuids::random_generator()())
{
}

const std::string& TaskComposerNode::getName() const { return name_; }

TaskComposerNodeType TaskComposerNode::getType() const { return type_; }

const boost::uuids::uuid& TaskComposerNode::getUUID() const { return uuid_; }

const std::vector<boost::uuids::uuid>& TaskComposerNode::getEdges() const { return edges_; }

bool TaskComposerNode::operator==(const TaskComposerNode& rhs) const
{
  bool equal = true;
  equal &= name_ == rhs.name_;
  equal &= type_ == rhs.type_;
  equal &= uuid_ == rhs.uuid_;
  equal &= edges_ == rhs.edges_;
  return equal;
}
bool TaskComposerNode::operator!=(const TaskComposerNode& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerNode::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name_);
  ar& boost::serialization::make_nvp("type", type_);
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("edges", edges_);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNode)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNode)

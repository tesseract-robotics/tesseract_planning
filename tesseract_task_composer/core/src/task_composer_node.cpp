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

#include <tesseract_task_composer/core/task_composer_node.h>

namespace tesseract_planning
{
TaskComposerNode::TaskComposerNode(std::string name, TaskComposerNodeType type, bool conditional)
  : name_(std::move(name))
  , type_(type)
  , uuid_(boost::uuids::random_generator()())
  , uuid_str_(boost::uuids::to_string(uuid_))
  , conditional_(conditional)
{
}

TaskComposerNode::TaskComposerNode(std::string name, TaskComposerNodeType type, const YAML::Node& config)
  : TaskComposerNode::TaskComposerNode(std::move(name), type)
{
  try
  {
    if (YAML::Node n = config["conditional"])
      conditional_ = n.as<bool>();

    if (YAML::Node n = config["inputs"])
    {
      if (n.IsSequence())
        input_keys_ = n.as<std::vector<std::string>>();
      else
        input_keys_ = { n.as<std::string>() };
    }

    if (YAML::Node n = config["outputs"])
    {
      if (n.IsSequence())
        output_keys_ = n.as<std::vector<std::string>>();
      else
        output_keys_ = { n.as<std::string>() };
    }
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TaskComposerNode: Failed to parse yaml config data! Details: " + std::string(e.what()));
  }
}

void TaskComposerNode::setName(const std::string& name) { name_ = name; }
const std::string& TaskComposerNode::getName() const { return name_; }

TaskComposerNodeType TaskComposerNode::getType() const { return type_; }

const boost::uuids::uuid& TaskComposerNode::getUUID() const { return uuid_; }

const std::string& TaskComposerNode::getUUIDString() const { return uuid_str_; }

const boost::uuids::uuid& TaskComposerNode::getParentUUID() const { return parent_uuid_; }

bool TaskComposerNode::isConditional() const { return conditional_; }

const std::vector<boost::uuids::uuid>& TaskComposerNode::getOutboundEdges() const { return outbound_edges_; }

const std::vector<boost::uuids::uuid>& TaskComposerNode::getInboundEdges() const { return inbound_edges_; }

void TaskComposerNode::setInputKeys(const std::vector<std::string>& input_keys) { input_keys_ = input_keys; }

const std::vector<std::string>& TaskComposerNode::getInputKeys() const { return input_keys_; }

void TaskComposerNode::setOutputKeys(const std::vector<std::string>& output_keys) { output_keys_ = output_keys; }

const std::vector<std::string>& TaskComposerNode::getOutputKeys() const { return output_keys_; }

void TaskComposerNode::renameInputKeys(const std::map<std::string, std::string>& input_keys)
{
  for (const auto& key : input_keys)
    std::replace(input_keys_.begin(), input_keys_.end(), key.first, key.second);
}

void TaskComposerNode::renameOutputKeys(const std::map<std::string, std::string>& output_keys)
{
  for (const auto& key : output_keys)
    std::replace(output_keys_.begin(), output_keys_.end(), key.first, key.second);
}

void TaskComposerNode::setConditional(bool enable) { conditional_ = enable; }

std::string TaskComposerNode::dump(std::ostream& os,
                                   const TaskComposerNode* /*parent*/,
                                   const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map) const
{
  const std::string tmp = toString(uuid_, "node_");

  std::string color{ "white" };
  int return_value = -1;

  auto it = results_map.find(uuid_);
  if (it != results_map.end())
  {
    return_value = it->second->return_value;
    if (!it->second->isAborted())
      color = it->second->color;
  }

  if (conditional_)
  {
    os << std::endl << tmp << " [shape=diamond, label=\"" << name_ << "\\n(" << uuid_str_ << ")";

    os << "\\n Inputs: [";
    for (std::size_t i = 0; i < input_keys_.size(); ++i)
    {
      os << input_keys_[i];
      if (i < input_keys_.size() - 1)
        os << ", ";
    }
    os << "]";

    os << "\\n Outputs: [";
    for (std::size_t i = 0; i < output_keys_.size(); ++i)
    {
      os << output_keys_[i];
      if (i < output_keys_.size() - 1)
        os << ", ";
    }
    os << "]";

    if (it != results_map.end())
    {
      os << "\\nTime: " << std::fixed << std::setprecision(3) << it->second->elapsed_time << "s"
         << "\\n`" << it->second->message << "`";
    }
    os << "\", color=black, fillcolor=" << color << ", style=filled];\n";

    for (std::size_t i = 0; i < outbound_edges_.size(); ++i)
    {
      std::string line_type = (return_value == static_cast<int>(i)) ? "bold" : "dashed";
      os << tmp << " -> " << toString(outbound_edges_[i], "node_") << " [style=" << line_type << ", label=\"["
         << std::to_string(i) << "]\""
         << "];\n";
    }
  }
  else
  {
    os << std::endl << tmp << " [label=\"" << name_ << "\\n(" << uuid_str_ << ")";

    os << "\\n Inputs: [";
    for (std::size_t i = 0; i < input_keys_.size(); ++i)
    {
      os << input_keys_[i];
      if (i < input_keys_.size() - 1)
        os << ", ";
    }
    os << "]";

    os << "\\n Outputs: [";
    for (std::size_t i = 0; i < output_keys_.size(); ++i)
    {
      os << output_keys_[i];
      if (i < output_keys_.size() - 1)
        os << ", ";
    }
    os << "]";

    if (it != results_map.end())
    {
      os << "\\nTime: " << std::fixed << std::setprecision(3) << it->second->elapsed_time << "s"
         << "\\n'" << it->second->message << "'";
    }
    os << "\", color=black, fillcolor=" << color << ", style=filled];\n";

    for (const auto& edge : outbound_edges_)
      os << tmp << " -> " << toString(edge, "node_") << ";\n";
  }

  if (it == results_map.end())
    return {};

  return it->second->dotgraph;
}

bool TaskComposerNode::operator==(const TaskComposerNode& rhs) const
{
  bool equal = true;
  equal &= name_ == rhs.name_;
  equal &= type_ == rhs.type_;
  equal &= uuid_ == rhs.uuid_;
  equal &= uuid_str_ == rhs.uuid_str_;
  equal &= parent_uuid_ == rhs.parent_uuid_;
  equal &= outbound_edges_ == rhs.outbound_edges_;
  equal &= inbound_edges_ == rhs.inbound_edges_;
  equal &= input_keys_ == rhs.input_keys_;
  equal &= output_keys_ == rhs.output_keys_;
  equal &= conditional_ == rhs.conditional_;
  return equal;
}
bool TaskComposerNode::operator!=(const TaskComposerNode& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerNode::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name_);
  ar& boost::serialization::make_nvp("type", type_);
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("uuid_str", uuid_str_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("outbound_edges", outbound_edges_);
  ar& boost::serialization::make_nvp("inbound_edges", inbound_edges_);
  ar& boost::serialization::make_nvp("input_keys", input_keys_);
  ar& boost::serialization::make_nvp("output_keys", output_keys_);
  ar& boost::serialization::make_nvp("conditional", conditional_);
}

std::string TaskComposerNode::toString(const boost::uuids::uuid& u, const std::string& prefix)
{
  std::string result;
  result.reserve(36);

  std::size_t i = 0;
  for (const auto* it_data = u.begin(); it_data != u.end(); ++it_data, ++i)
  {
    const size_t hi = ((*it_data) >> 4) & 0x0F;
    result += boost::uuids::detail::to_char(hi);

    const size_t lo = (*it_data) & 0x0F;
    result += boost::uuids::detail::to_char(lo);
  }
  return (prefix + result);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerNode)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerNode)

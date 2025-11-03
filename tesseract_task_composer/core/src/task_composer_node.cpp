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
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <yaml-cpp/yaml.h>
#include <console_bridge/console.h>
#include <tesseract_common/stopwatch.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/yaml_extensions.h>

namespace tesseract_planning
{
TaskComposerNode::TaskComposerNode(std::string name,
                                   TaskComposerNodeType type,
                                   TaskComposerNodePorts ports,
                                   bool conditional)
  : name_(std::move(name))
  , ns_(name_)
  , type_(type)
  , uuid_(boost::uuids::random_generator()())
  , uuid_str_(boost::uuids::to_string(uuid_))
  , parent_uuid_str_(boost::uuids::to_string(parent_uuid_))
  , conditional_(conditional)
  , ports_(std::move(ports))
{
}

TaskComposerNode::TaskComposerNode(std::string name,
                                   TaskComposerNodeType type,
                                   TaskComposerNodePorts ports,
                                   const YAML::Node& config)
  : TaskComposerNode::TaskComposerNode(std::move(name), type, std::move(ports))
{
  try
  {
    ns_ = config["namespace"].IsDefined() ? config["namespace"].as<std::string>() : name_;

    if (YAML::Node n = config["conditional"])
      conditional_ = n.as<bool>();

    if (YAML::Node n = config["inputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("TaskComposerNode, inputs must be a map type");

      input_keys_ = n.as<TaskComposerKeys>();
    }

    if (YAML::Node n = config["outputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("TaskComposerNode, outputs must be a map type");

      output_keys_ = n.as<TaskComposerKeys>();
    }
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TaskComposerNode: Failed to parse yaml config data! Details: " + std::string(e.what()));
  }

  if (type != TaskComposerNodeType::GRAPH && type != TaskComposerNodeType::PIPELINE)
    validatePorts();
}

int TaskComposerNode::run(TaskComposerContext& context, OptionalTaskComposerExecutor executor) const
{
  auto start_time = std::chrono::system_clock::now();
  if (context.isAborted())
  {
    TaskComposerNodeInfo info(*this);
    info.start_time = start_time;
    info.return_value = 0;
    info.color = "grey";
    info.status_code = 0;
    info.status_message = "Aborted";
    info.aborted = true;
    context.task_infos->addInfo(info);
    return 0;
  }

  tesseract_common::Stopwatch stopwatch;
  TaskComposerNodeInfo results;
  stopwatch.start();

  // Set the root node in task_infos
  if (parent_uuid_.is_nil())
    context.task_infos->setRootNode(uuid_);

  bool exception_occured{ false };
  try
  {
    results = runImpl(context, executor);
  }
  catch (const std::exception& e)
  {
    exception_occured = true;
    results = TaskComposerNodeInfo(*this);
    results.color = "red";
    results.status_code = -1;
    results.status_message = "Exception thrown: " + std::string(e.what());
    results.return_value = 0;
  }
  stopwatch.stop();
  results.input_keys = input_keys_;
  results.output_keys = output_keys_;
  results.start_time = start_time;
  results.elapsed_time = stopwatch.elapsedSeconds();

  int value = results.return_value;
  assert(value >= 0);

  // Call abort if exception or required and is a task
  if ((exception_occured && !context.isAborted()) ||
      (type_ == TaskComposerNodeType::TASK && trigger_abort_ && !context.isAborted()))
  {
    results.status_message += " (Abort Triggered)";
    context.abort(uuid_);
  }

  context.task_infos->addInfo(results);
  return value;
}

void TaskComposerNode::setName(const std::string& name) { name_ = name; }
const std::string& TaskComposerNode::getName() const { return name_; }

void TaskComposerNode::setNamespace(const std::string& ns) { ns_ = ns; }
const std::string& TaskComposerNode::getNamespace() const { return ns_; }

TaskComposerNodeType TaskComposerNode::getType() const { return type_; }

const boost::uuids::uuid& TaskComposerNode::getUUID() const { return uuid_; }

const std::string& TaskComposerNode::getUUIDString() const { return uuid_str_; }

const boost::uuids::uuid& TaskComposerNode::getParentUUID() const { return parent_uuid_; }

const std::string& TaskComposerNode::getParentUUIDString() const { return parent_uuid_str_; }

bool TaskComposerNode::isConditional() const { return conditional_; }

void TaskComposerNode::validatePorts() const
{
  const auto& input_keys = input_keys_.data();
  const auto& output_keys = output_keys_.data();

  // Check for required ports
  for (const auto& [port, type] : ports_.input_required)
  {
    auto it = input_keys.find(port);
    if (it == input_keys.end())
    {
      std::string msg;
      msg.append(name_);
      msg.append(", missing required input port '");
      msg.append(port);
      msg.append(":");
      msg.append((static_cast<bool>(type)) ? "Multiple" : "Single");
      msg.append("'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }

    if (it->second.index() != type)
    {
      std::string msg;
      msg.append(name_);
      msg.append(", required input port is wrong type'");
      msg.append(port);
      msg.append(":");
      msg.append((static_cast<bool>(type)) ? "Multiple" : "Single");
      msg.append("'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }

    if (static_cast<bool>(type) && std::get<std::vector<std::string>>(it->second).empty())
    {
      std::string msg;
      msg.append(name_);
      msg.append(", required input port container is empty'");
      msg.append(port);
      msg.append(":Multiple'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }
  }

  for (const auto& [port, type] : ports_.output_required)
  {
    auto it = output_keys.find(port);
    if (it == output_keys.end())
    {
      std::string msg;
      msg.append(name_);
      msg.append(", missing required output port '");
      msg.append(port);
      msg.append(":");
      msg.append((static_cast<bool>(type)) ? "Multiple" : "Single");
      msg.append("'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }

    if (it->second.index() != type)
    {
      std::string msg;
      msg.append(name_);
      msg.append(", required output port is wrong type'");
      msg.append(port);
      msg.append(":");
      msg.append((static_cast<bool>(type)) ? "Multiple" : "Single");
      msg.append("'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }

    if (static_cast<bool>(type) && std::get<std::vector<std::string>>(it->second).empty())
    {
      std::string msg;
      msg.append(name_);
      msg.append(", required output port container is empty'");
      msg.append(port);
      msg.append(":Multiple'. Supported Ports:\n");
      msg.append(ports_.toString());
      throw std::runtime_error(msg);
    }
  }

  // Check for extra ports that do not belong
  for (const auto& [port, key] : input_keys)
  {
    {
      auto it = ports_.input_required.find(port);
      if ((it != ports_.input_required.end()) && (key.index() == it->second))
        continue;
    }

    {
      auto it = ports_.input_optional.find(port);
      if ((it != ports_.input_optional.end()) && (key.index() == it->second))
      {
        if ((it != ports_.input_optional.end()) && static_cast<bool>(it->second) &&
            std::get<std::vector<std::string>>(key).empty())
        {
          std::string msg;
          msg.append(name_);
          msg.append(", optional input port container is empty'");
          msg.append(port);
          msg.append(":Container'. Supported Ports:\n");
          msg.append(ports_.toString());
          throw std::runtime_error(msg);
        }

        continue;
      }
    }

    std::string msg;
    msg.append(name_);
    msg.append(", invalid input port defined '");
    msg.append(port);
    msg.append(":");
    msg.append((key.index() == 1) ? "Multiple" : "Single");
    msg.append("'. Supported Ports:\n");
    msg.append(ports_.toString());
    throw std::runtime_error(msg);
  }

  for (const auto& [port, key] : output_keys)
  {
    {
      auto it = ports_.output_required.find(port);
      if ((it != ports_.output_required.end()) && (key.index() == it->second))
        continue;
    }

    {
      auto it = ports_.output_optional.find(port);
      if ((it != ports_.output_optional.end()) && (key.index() == it->second))
      {
        if ((it != ports_.output_optional.end()) && static_cast<bool>(it->second) &&
            std::get<std::vector<std::string>>(key).empty())
        {
          std::string msg;
          msg.append(name_);
          msg.append(", optional output port container is empty'");
          msg.append(port);
          msg.append(":Container'. Supported Ports:\n");
          msg.append(ports_.toString());
          throw std::runtime_error(msg);
        }

        continue;
      }
    }

    std::string msg;
    msg.append(name_);
    msg.append(", invalid output port defined '");
    msg.append(port);
    msg.append(":");
    msg.append((key.index() == 1) ? "Multiple" : "Single");
    msg.append("'. Supported Ports:\n");
    msg.append(ports_.toString());
    throw std::runtime_error(msg);
  }
}

const std::vector<boost::uuids::uuid>& TaskComposerNode::getOutboundEdges() const { return outbound_edges_; }

const std::vector<boost::uuids::uuid>& TaskComposerNode::getInboundEdges() const { return inbound_edges_; }

void TaskComposerNode::setInputKeys(const TaskComposerKeys& input_keys) { input_keys_ = input_keys; }

const TaskComposerKeys& TaskComposerNode::getInputKeys() const { return input_keys_; }

void TaskComposerNode::setOutputKeys(const TaskComposerKeys& output_keys) { output_keys_ = output_keys; }

const TaskComposerKeys& TaskComposerNode::getOutputKeys() const { return output_keys_; }

TaskComposerNodePorts TaskComposerNode::getPorts() const { return ports_; }

std::string TaskComposerNode::getDotgraph(const ResultsMap& results_map) const
{
  try
  {
    // Save dot graph
    std::stringstream dotgraph;
    dump(dotgraph, nullptr, results_map);
    return dotgraph.str();
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to generated DOT Graph: '%s'!", e.what());
  }

  return {};
}

bool TaskComposerNode::saveDotgraph(const std::string& filepath, const ResultsMap& results_map) const
{
  try
  {
    // Save dot graph
    std::ofstream os;
    os.open(filepath);
    dump(os, nullptr, results_map);
    os.close();
    return true;
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to save DOT Graph: '%s'!", e.what());
  }

  return false;
}

void TaskComposerNode::setConditional(bool enable) { conditional_ = enable; }

std::string TaskComposerNode::dump(std::ostream& os,
                                   const TaskComposerNode* /*parent*/,
                                   const ResultsMap& results_map) const
{
  const std::string tmp = toString(uuid_, "node_");

  std::string color{ "white" };
  int return_value = -1;

  auto it = results_map.find(uuid_);
  if (it != results_map.end())
  {
    return_value = it->second.return_value;
    if (!it->second.aborted)
      color = it->second.color;
  }

  if (conditional_)
  {
    os << "\n" << tmp << " [shape=diamond, nojustify=true label=\"" << name_ << "\\n";
    os << "Type: " << boost::core::demangle(typeid(*this).name()) << "\\l";
    os << "UUID: " << uuid_str_ << "\\l";
    os << "Namespace: " << ns_ << "\\l";
    os << "Inputs:\\l" << input_keys_;
    os << "Outputs:\\l" << output_keys_;

    if (it != results_map.end())
    {
      os << "Time: " << std::fixed << std::setprecision(3) << it->second.elapsed_time << "s\\l"
         << "Status Code: " << std::to_string(it->second.status_code) << "\\l"
         << "Status Msg: " << it->second.status_message << "\\l";
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
    os << "\n" << tmp << " [nojustify=true label=\"" << name_ << "\\n";
    os << "Type: " << boost::core::demangle(typeid(*this).name()) << "\\l";
    os << "UUID: " << uuid_str_ << "\\l";
    os << "Namespace: " << ns_ << "\\l";
    os << "Inputs:\\l" << input_keys_;
    os << "Outputs:\\l" << output_keys_;

    if (it != results_map.end())
    {
      os << "Time: " << std::fixed << std::setprecision(3) << it->second.elapsed_time << "s\\l"
         << "Status Code: " << std::to_string(it->second.status_code) << "\\l"
         << "Status Msg: " << it->second.status_message << "\\l";
    }
    os << "\", color=black, fillcolor=" << color << ", style=filled];\n";

    for (const auto& edge : outbound_edges_)
      os << tmp << " -> " << toString(edge, "node_") << ";\n";
  }

  if (it == results_map.end())
    return {};

  return it->second.dotgraph;
}

std::string TaskComposerNode::toString(const boost::uuids::uuid& u, const std::string& prefix)
{
  auto result = boost::uuids::to_string(u);
  boost::replace_all(result, "-", "");
  return (prefix + result);
}

TaskComposerDataStorage::Ptr TaskComposerNode::getDataStorage(const TaskComposerContext& context) const
{
  if (parent_uuid_.is_nil())
    return context.data_storage;

  tesseract_common::AnyPoly data_storage_any = context.data_storage->getData(parent_uuid_str_);
  if (data_storage_any.isNull())
    throw std::runtime_error("TaskComposerNode, unable to find parent data storage!");

  return data_storage_any.as<TaskComposerDataStorage::Ptr>();
}

template <>
tesseract_common::AnyPoly TaskComposerNode::getData(const TaskComposerContext& context,
                                                    const std::string& port,
                                                    bool required) const
{
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);
  auto it = input_keys_.data().find(port);
  if (it == input_keys_.data().end())
  {
    if (required)
      throw std::runtime_error(name_ + ", required key does not exist for the provided name: " + port);

    return {};
  }

  const auto& key = std::get<std::string>(it->second);
  auto data = data_storage->getData(key);
  if (data.isNull() && required)
    throw std::runtime_error(name_ + ", required data is missing: " + port + ":" + key);

  return data;
}

template <>
std::vector<tesseract_common::AnyPoly> TaskComposerNode::getData(const TaskComposerContext& context,
                                                                 const std::string& port,
                                                                 bool required) const
{
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);
  auto it = input_keys_.data().find(port);
  if (it == input_keys_.data().end())
  {
    if (required)
      throw std::runtime_error(name_ + ", required key does not exist for the provided name: " + port);

    return {};
  }

  const auto& vs = std::get<std::vector<std::string>>(it->second);
  std::vector<tesseract_common::AnyPoly> data_container;
  for (const auto& key : vs)
  {
    auto data = data_storage->getData(key);
    if (data.isNull() && required)
    {
      std::string msg(name_);
      msg.append(", required data is missing: ");
      msg.append(port);
      msg.append(":");
      msg.append(key);
      throw std::runtime_error(msg);
    }

    data_container.push_back(data);
  }

  return data_container;
}

void TaskComposerNode::setData(TaskComposerContext& context,
                               const std::string& port,
                               tesseract_common::AnyPoly data,
                               bool required) const
{
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);
  auto it = output_keys_.data().find(port);
  if (it == output_keys_.data().end())
  {
    if (required)
      throw std::runtime_error(name_ + ", output key does not exist for the provided name: " + port);

    return;
  }

  const auto& key = std::get<std::string>(it->second);

  data_storage->setData(key, std::move(data));
}

void TaskComposerNode::setData(TaskComposerContext& context,
                               const std::string& port,
                               const std::vector<tesseract_common::AnyPoly>& data,
                               bool required) const
{
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);
  auto it = output_keys_.data().find(port);
  if (it == output_keys_.data().end())
  {
    if (required)
      throw std::runtime_error(name_ + ", output key does not exist for the provided name: " + port);

    return;
  }

  const auto& vs = std::get<std::vector<std::string>>(it->second);
  if (vs.size() != data.size())
    throw std::runtime_error(
        name_ + ", output container and assigned data are not the same size for the provided name: " + port);

  for (std::size_t i = 0; i < vs.size(); ++i)
    data_storage->setData(vs[i], data[i]);
}

}  // namespace tesseract_planning

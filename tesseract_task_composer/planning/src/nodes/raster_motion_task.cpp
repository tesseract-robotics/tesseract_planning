/**
 * @file raster_motion_task.cpp
 * @brief Raster motion task with transitions
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
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>

#include <tesseract_common/serialization.h>
#include <tesseract_common/yaml_utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/raster_motion_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_state_task.h>

#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/yaml_utils.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_environment/environment.h>

namespace
{
tesseract_planning::RasterMotionTask::TaskFactoryResults
createTask(const YAML::Node& config,
           const std::string& parent_name,
           const std::string& name,
           const tesseract_planning::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  static const std::string inout_port{ "program" };
  tesseract_planning::RasterMotionTask::TaskFactoryResults tr;
  tr.node = loadSubTask(parent_name, name, config, plugin_factory);
  tr.node->setConditional(false);
  tr.input_key = tr.node->getInputKeys().get(inout_port) + std::to_string(index);
  tr.output_key = tr.node->getOutputKeys().get(inout_port) + std::to_string(index);

  auto& graph_node = static_cast<tesseract_planning::TaskComposerGraph&>(*tr.node);
  tesseract_planning::TaskComposerKeys override_input_keys;
  tesseract_planning::TaskComposerKeys override_output_keys;
  override_input_keys.add(inout_port, tr.input_key);
  override_output_keys.add(inout_port, tr.output_key);
  graph_node.setOverrideInputKeys(override_input_keys);
  graph_node.setOverrideOutputKeys(override_output_keys);

  return tr;
}
}  // namespace

namespace tesseract_planning
{
// Requried
const std::string RasterMotionTask::INOUT_PROGRAM_PORT = "program";
const std::string RasterMotionTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string RasterMotionTask::INPUT_PROFILES_PORT = "profiles";

RasterMotionTask::RasterMotionTask() : TaskComposerTask("RasterMotionTask", RasterMotionTask::ports(), true) {}
RasterMotionTask::RasterMotionTask(std::string name,
                                   std::string input_program_key,
                                   std::string input_environment_key,
                                   std::string input_profiles_key,
                                   std::string output_program_key,
                                   bool conditional,
                                   TaskFactory freespace_task_factory,
                                   TaskFactory raster_task_factory,
                                   TaskFactory transition_task_factory)
  : TaskComposerTask(std::move(name), RasterMotionTask::ports(), conditional)
  , freespace_task_factory_(std::move(freespace_task_factory))
  , raster_task_factory_(std::move(raster_task_factory))
  , transition_task_factory_(std::move(transition_task_factory))
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

RasterMotionTask::RasterMotionTask(std::string name,
                                   const YAML::Node& config,
                                   const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), RasterMotionTask::ports(), config)
{
  static const std::set<std::string> tasks_expected_keys{ "task", "class", "config", "override" };
  static const std::string freespace_key{ "freespace" };
  static const std::string raster_key{ "raster" };
  static const std::string transition_key{ "transition" };

  if (YAML::Node freespace_config = config[freespace_key])
  {
    tesseract_common::checkForUnknownKeys(freespace_config, tasks_expected_keys);
    validateSubTask(name_, freespace_key, freespace_config);

    freespace_task_factory_ = [freespace_config, &plugin_factory](
                                  const std::string& parent_name, const std::string& name, std::size_t index) {
      return createTask(freespace_config, parent_name, name, plugin_factory, index);
    };
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'freespace' entry");
  }

  if (YAML::Node raster_config = config[raster_key])
  {
    tesseract_common::checkForUnknownKeys(raster_config, tasks_expected_keys);
    validateSubTask(name_, raster_key, raster_config);

    raster_task_factory_ =
        [raster_config, &plugin_factory](const std::string& parent_name, const std::string& name, std::size_t index) {
          return createTask(raster_config, parent_name, name, plugin_factory, index);
        };
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'raster' entry");
  }

  if (YAML::Node transition_config = config[transition_key])
  {
    tesseract_common::checkForUnknownKeys(transition_config, tasks_expected_keys);
    validateSubTask(name_, transition_key, transition_config);

    transition_task_factory_ = [transition_config, &plugin_factory](
                                   const std::string& parent_name, const std::string& name, std::size_t index) {
      return createTask(transition_config, parent_name, name, plugin_factory, index);
    };
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'transition' entry");
  }
}

TaskComposerNodePorts RasterMotionTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

bool RasterMotionTask::operator==(const RasterMotionTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool RasterMotionTask::operator!=(const RasterMotionTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterMotionTask::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

TaskComposerNodeInfo RasterMotionTask::runImpl(TaskComposerContext& context,
                                               OptionalTaskComposerExecutor executor) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(context, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
  {
    info.status_code = 0;
    info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    info.return_value = 0;
    return info;
  }

  std::shared_ptr<const tesseract_environment::Environment> env =
      env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>()->clone();
  info.data_storage.setData("environment", env);

  auto input_data_poly = getData(context, INOUT_PROGRAM_PORT);
  try
  {
    checkTaskInput(input_data_poly);
  }
  catch (const std::exception& e)
  {
    info.status_message = e.what();
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }
  auto& program = input_data_poly.template as<CompositeInstruction>();
  tesseract_common::ManipulatorInfo program_manip_info = program.getManipulatorInfo();

  // Create Sub Graph Task
  TaskComposerGraph task_graph(name_ + " (Subgraph)", uuid_);

  // Create Sub Graph Task Input and Output Keys
  // Must copy the existing parent input/output keys, but remove program port key which will get assigned later.
  TaskComposerKeys task_input_keys{ input_keys_ };
  TaskComposerKeys task_output_keys{ output_keys_ };
  task_input_keys.remove(INOUT_PROGRAM_PORT);
  task_output_keys.remove(INOUT_PROGRAM_PORT);

  // Create a sub graph data storage and copy the input data relevant to this graph.
  auto task_graph_data_storage = std::make_shared<TaskComposerDataStorage>(uuid_str_);
  task_graph_data_storage->copyAsInputData(*context.data_storage, task_input_keys, {});

  // Create container to store the sub graph program port keys
  std::vector<std::string> input_keys;
  std::vector<std::string> output_keys;
  input_keys.reserve(program.size());
  output_keys.reserve(program.size());

  // Start Task
  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> raster_tasks;
  raster_tasks.reserve(program.size());

  // Generate all of the raster tasks. They don't depend on anything
  std::size_t raster_idx = 0;
  for (std::size_t idx = 1; idx < program.size() - 1; idx += 2)
  {
    // Get Raster program
    auto raster_input = program[idx].template as<CompositeInstruction>();

    // Set the manipulator info
    raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));

    // Get Start Plan Instruction
    const InstructionPoly& pre_input_instruction = program[idx - 1];
    assert(pre_input_instruction.isCompositeInstruction());
    const auto& tci = pre_input_instruction.as<CompositeInstruction>();
    const auto* li = tci.getLastMoveInstruction();
    assert(li != nullptr);
    raster_input.insert(raster_input.begin(), *li);

    const std::string task_name = "Raster #" + std::to_string(raster_idx + 1) + ": " + raster_input.getDescription();
    auto raster_results = raster_task_factory_(name_, task_name, raster_idx + 1);
    auto raster_uuid = task_graph.addNode(std::move(raster_results.node));
    raster_tasks.emplace_back(raster_uuid, std::make_pair(raster_results.input_key, raster_results.output_key));
    input_keys.push_back(raster_results.input_key);
    output_keys.push_back(raster_results.output_key);
    task_graph_data_storage->setData(raster_results.input_key, raster_input);

    task_graph.addEdges(start_uuid, { raster_uuid });

    raster_idx++;
  }

  // Loop over all transitions
  std::vector<std::pair<std::string, std::string>> transition_keys;
  transition_keys.reserve(program.size());
  std::size_t transition_idx = 0;
  for (std::size_t idx = 2; idx < program.size() - 2; idx += 2)
  {
    // Get transition program
    auto transition_input = program[idx].template as<CompositeInstruction>();

    // Set the manipulator info
    transition_input.setManipulatorInfo(transition_input.getManipulatorInfo().getCombined(program_manip_info));

    // Get Start Plan Instruction
    const InstructionPoly& pre_input_instruction = program[idx - 1];
    assert(pre_input_instruction.isCompositeInstruction());
    const auto& tci = pre_input_instruction.as<CompositeInstruction>();
    const auto* li = tci.getLastMoveInstruction();
    assert(li != nullptr);
    transition_input.insert(transition_input.begin(), *li);

    const std::string task_name =
        "Transition #" + std::to_string(transition_idx + 1) + ": " + transition_input.getDescription();
    auto transition_results = transition_task_factory_(name_, task_name, transition_idx + 1);
    auto transition_uuid = task_graph.addNode(std::move(transition_results.node));
    transition_keys.emplace_back(transition_results.input_key, transition_results.output_key);

    const auto& prev = raster_tasks[transition_idx];
    const auto& next = raster_tasks[transition_idx + 1];
    const auto& prev_output = prev.second.second;
    const auto& next_output = next.second.second;
    auto transition_mux_task = std::make_unique<UpdateStartAndEndStateTask>("UpdateStartAndEndStateTask",
                                                                            transition_results.input_key,
                                                                            prev_output,
                                                                            next_output,
                                                                            transition_results.input_key,
                                                                            false);
    auto transition_mux_uuid = task_graph.addNode(std::move(transition_mux_task));

    input_keys.push_back(transition_results.input_key);
    output_keys.push_back(transition_results.output_key);
    task_graph_data_storage->setData(transition_results.input_key, transition_input);

    task_graph.addEdges(transition_mux_uuid, { transition_uuid });
    task_graph.addEdges(prev.first, { transition_mux_uuid });
    task_graph.addEdges(next.first, { transition_mux_uuid });

    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  auto from_start_input = program[0].template as<CompositeInstruction>();
  from_start_input.setManipulatorInfo(from_start_input.getManipulatorInfo().getCombined(program_manip_info));

  auto from_start_results = freespace_task_factory_(name_, "From Start: " + from_start_input.getDescription(), 0);
  auto from_start_pipeline_uuid = task_graph.addNode(std::move(from_start_results.node));

  const auto& first_raster_output_key = raster_tasks[0].second.second;
  auto update_end_state_task = std::make_unique<UpdateEndStateTask>(
      "UpdateEndStateTask", from_start_results.input_key, first_raster_output_key, from_start_results.input_key, false);
  auto update_end_state_uuid = task_graph.addNode(std::move(update_end_state_task));

  input_keys.push_back(from_start_results.input_key);
  output_keys.push_back(from_start_results.output_key);
  task_graph_data_storage->setData(from_start_results.input_key, from_start_input);

  task_graph.addEdges(update_end_state_uuid, { from_start_pipeline_uuid });
  task_graph.addEdges(raster_tasks[0].first, { update_end_state_uuid });

  // Plan to_end - preceded by the last raster
  auto to_end_input = program.back().template as<CompositeInstruction>();
  to_end_input.setManipulatorInfo(to_end_input.getManipulatorInfo().getCombined(program_manip_info));

  // Get Start Plan Instruction
  const InstructionPoly& pre_input_instruction = program[program.size() - 2];
  assert(pre_input_instruction.isCompositeInstruction());
  const auto& tci = pre_input_instruction.as<CompositeInstruction>();
  const auto* li = tci.getLastMoveInstruction();
  assert(li != nullptr);
  to_end_input.insert(to_end_input.begin(), *li);

  auto to_end_results = freespace_task_factory_(name_, "To End: " + to_end_input.getDescription(), program.size());
  auto to_end_pipeline_uuid = task_graph.addNode(std::move(to_end_results.node));

  const auto& last_raster_output_key = raster_tasks.back().second.second;
  auto update_start_state_task = std::make_unique<UpdateStartStateTask>(
      "UpdateStartStateTask", to_end_results.input_key, last_raster_output_key, to_end_results.input_key, false);
  auto update_start_state_uuid = task_graph.addNode(std::move(update_start_state_task));

  input_keys.push_back(to_end_results.input_key);
  output_keys.push_back(to_end_results.output_key);
  task_graph_data_storage->setData(to_end_results.input_key, to_end_input);

  task_graph.addEdges(update_start_state_uuid, { to_end_pipeline_uuid });
  task_graph.addEdges(raster_tasks.back().first, { update_start_state_uuid });

  if (!executor.has_value())
    throw std::runtime_error("RasterMotionTask, executor is null!");

  // Set sub graph input and output keys
  task_input_keys.add(INOUT_PROGRAM_PORT, input_keys);
  task_output_keys.add(INOUT_PROGRAM_PORT, output_keys);
  task_graph.setInputKeys(task_input_keys);
  task_graph.setOutputKeys(task_output_keys);

  // Store sub data storage in parent data storage
  context.data_storage->setData(uuid_str_, task_graph_data_storage);

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, context.shared_from_this());
  future->wait();

  auto info_map = context.task_infos->getInfoMap();
  if (context.dotgraph)
  {
    std::stringstream dot_graph;
    dot_graph << "subgraph cluster_" << toString(uuid_) << " {\n color=black;\n label = \"" << name_ << "\\n("
              << uuid_str_ << ")\";\n";
    task_graph.dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info.dotgraph = dot_graph.str();
  }

  if (context.isAborted())
  {
    info.status_message = "Raster subgraph failed";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  program.clear();
  program.emplace_back(task_graph_data_storage->getData(from_start_results.output_key).as<CompositeInstruction>());
  for (std::size_t i = 0; i < raster_tasks.size(); ++i)
  {
    const auto& raster_output_key = raster_tasks[i].second.second;
    CompositeInstruction segment = task_graph_data_storage->getData(raster_output_key).as<CompositeInstruction>();
    segment.erase(segment.begin());
    program.emplace_back(segment);

    if (i < raster_tasks.size() - 1)
    {
      const auto& transition_output_key = transition_keys[i].second;
      CompositeInstruction transition =
          task_graph_data_storage->getData(transition_output_key).as<CompositeInstruction>();
      transition.erase(transition.begin());
      program.emplace_back(transition);
    }
  }
  CompositeInstruction to_end = task_graph_data_storage->getData(to_end_results.output_key).as<CompositeInstruction>();
  to_end.erase(to_end.begin());
  program.emplace_back(to_end);

  setData(context, INOUT_PROGRAM_PORT, program);

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

void RasterMotionTask::checkTaskInput(const tesseract_common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("RasterMotionTask, input is null");

  if (input.getType() != std::type_index(typeid(CompositeInstruction)))
    throw std::runtime_error("RasterMotionTask, input is not a composite instruction");

  const auto& composite = input.as<CompositeInstruction>();

  // Check from_start
  if (!composite.at(0).isCompositeInstruction())
    throw std::runtime_error("RasterMotionTask, from_start should be a composite");

  // Check rasters and transitions
  for (std::size_t index = 1; index < composite.size() - 1; index++)
  {
    // Both rasters and transitions should be a composite
    if (!composite.at(index).isCompositeInstruction())
      throw std::runtime_error("RasterMotionTask, Both rasters and transitions should be a composite");
  }

  // Check to_end
  if (!composite.back().isCompositeInstruction())
    throw std::runtime_error("RasterMotionTask, to_end should be a composite");
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterMotionTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterMotionTask)

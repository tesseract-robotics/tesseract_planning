/**
 * @file raster_only_motion_task.h
 * @brief Plans raster paths only
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>

#include <tesseract_common/yaml_utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/raster_only_motion_task.h>
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
tesseract::task_composer::RasterOnlyMotionTask::TaskFactoryResults
createTask(const YAML::Node& config,
           const std::string& parent_name,
           const std::string& name,
           const tesseract::task_composer::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  static const std::string inout_port{ "program" };
  tesseract::task_composer::RasterOnlyMotionTask::TaskFactoryResults tr;
  tr.node = loadSubTask(parent_name, name, config, plugin_factory);
  tr.node->setConditional(false);
  tr.input_key = tr.node->getInputKeys().get(inout_port) + std::to_string(index);
  tr.output_key = tr.node->getOutputKeys().get(inout_port) + std::to_string(index);

  auto& graph_node = static_cast<tesseract::task_composer::TaskComposerGraph&>(*tr.node);
  tesseract::task_composer::TaskComposerKeys override_input_keys;
  tesseract::task_composer::TaskComposerKeys override_output_keys;
  override_input_keys.add(inout_port, tr.input_key);
  override_output_keys.add(inout_port, tr.output_key);
  graph_node.setOverrideInputKeys(override_input_keys);
  graph_node.setOverrideOutputKeys(override_output_keys);

  return tr;
}
}  // namespace

namespace tesseract::task_composer
{
// Requried
const std::string RasterOnlyMotionTask::INOUT_PROGRAM_PORT = "program";
const std::string RasterOnlyMotionTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string RasterOnlyMotionTask::INPUT_PROFILES_PORT = "profiles";

RasterOnlyMotionTask::RasterOnlyMotionTask()
  : TaskComposerTask("RasterOnlyMotionTask", RasterOnlyMotionTask::ports(), true)
{
}
RasterOnlyMotionTask::RasterOnlyMotionTask(std::string name,
                                           std::string input_program_key,
                                           std::string input_environment_key,
                                           std::string input_profiles_key,
                                           std::string output_program_key,
                                           bool is_conditional,
                                           TaskFactory raster_task_factory,
                                           TaskFactory transition_task_factory)
  : TaskComposerTask(std::move(name), RasterOnlyMotionTask::ports(), is_conditional)
  , raster_task_factory_(std::move(raster_task_factory))
  , transition_task_factory_(std::move(transition_task_factory))
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

RasterOnlyMotionTask::RasterOnlyMotionTask(std::string name,
                                           const YAML::Node& config,
                                           const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), RasterOnlyMotionTask::ports(), config)
{
  static const std::set<std::string> tasks_expected_keys{ "task", "class", "config", "override" };
  static const std::string raster_key{ "raster" };
  static const std::string transition_key{ "transition" };

  if (YAML::Node raster_config = config[raster_key])
  {
    tesseract::common::checkForUnknownKeys(raster_config, tasks_expected_keys);
    validateSubTask(name_, raster_key, raster_config);

    raster_task_factory_ =
        [raster_config, &plugin_factory](const std::string& parent_name, const std::string& name, std::size_t index) {
          return createTask(raster_config, parent_name, name, plugin_factory, index);
        };
  }
  else
  {
    throw std::runtime_error("RasterOnlyMotionTask: missing 'raster' entry");
  }

  if (YAML::Node transition_config = config[transition_key])
  {
    tesseract::common::checkForUnknownKeys(transition_config, tasks_expected_keys);
    validateSubTask(name_, transition_key, transition_config);

    transition_task_factory_ = [transition_config, &plugin_factory](
                                   const std::string& parent_name, const std::string& name, std::size_t index) {
      return createTask(transition_config, parent_name, name, plugin_factory, index);
    };
  }
  else
  {
    throw std::runtime_error("RasterOnlyMotionTask: missing 'transition' entry");
  }
}

TaskComposerNodePorts RasterOnlyMotionTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

TaskComposerNodeInfo RasterOnlyMotionTask::runImpl(TaskComposerContext& context,
                                                   OptionalTaskComposerExecutor executor) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(context, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract::environment::Environment>)))
  {
    info.status_code = 0;
    info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    info.return_value = 0;
    return info;
  }

  std::shared_ptr<const tesseract::environment::Environment> env =
      env_poly.as<std::shared_ptr<const tesseract::environment::Environment>>()->clone();
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

  auto& program = input_data_poly.template as<tesseract::command_language::CompositeInstruction>();
  tesseract::common::ManipulatorInfo program_manip_info = program.getManipulatorInfo();

  // Create Sub Graph Task Input and Output Keys
  // Must copy the existing parent input/output keys, but remove program port key which will get assigned later.
  TaskComposerGraph task_graph(name_ + " (Subgraph)", uuid_);
  TaskComposerKeys task_input_keys{ input_keys_ };
  TaskComposerKeys task_output_keys{ output_keys_ };
  task_input_keys.remove(INOUT_PROGRAM_PORT);
  task_output_keys.remove(INOUT_PROGRAM_PORT);

  // Create a sub graph data storage and copy the input data relevant to this graph.
  const TaskComposerDataStorage::Ptr parent_data_storage = getDataStorage(context);
  auto task_graph_data_storage = std::make_shared<TaskComposerDataStorage>(uuid_str_);
  task_graph_data_storage->copyAsInputData(*parent_data_storage, task_input_keys, {});

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
  for (std::size_t idx = 0; idx < program.size(); idx += 2)
  {
    // Get Raster program
    auto raster_input = program[idx].template as<tesseract::command_language::CompositeInstruction>();

    // Set the manipulator info
    raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));

    // Insert Start Plan Instruction
    if (idx > 0)
    {
      const tesseract::command_language::InstructionPoly& pre_input_instruction = program[idx - 1];
      assert(pre_input_instruction.isCompositeInstruction());
      const auto& tci = pre_input_instruction.as<tesseract::command_language::CompositeInstruction>();
      const auto* li = tci.getLastMoveInstruction();
      assert(li != nullptr);
      raster_input.insert(raster_input.begin(), *li);
    }

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
  for (std::size_t idx = 1; idx < program.size() - 1; idx += 2)
  {
    // Get transition program
    auto transition_input = program[idx].template as<tesseract::command_language::CompositeInstruction>();

    // Set the manipulator info
    transition_input.setManipulatorInfo(transition_input.getManipulatorInfo().getCombined(program_manip_info));

    const tesseract::command_language::InstructionPoly& pre_input_instruction = program[idx - 1];
    assert(pre_input_instruction.isCompositeInstruction());
    const auto& tci = pre_input_instruction.as<tesseract::command_language::CompositeInstruction>();
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

  if (!executor.has_value())
    throw std::runtime_error("RasterOnlyMotionTask, executor is null!");

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
              << uuid_str_ << ")\";";
    task_graph.dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info.dotgraph = dot_graph.str();
  }

  if (context.isAborted())
  {
    info.status_message = "Raster only subgraph failed";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  program.clear();
  for (std::size_t i = 0; i < raster_tasks.size(); ++i)
  {
    auto segment = task_graph_data_storage->getData(raster_tasks[i].second.second)
                       .as<tesseract::command_language::CompositeInstruction>();
    if (i != 0)
      segment.erase(segment.begin());

    program.emplace_back(segment);

    if (i < raster_tasks.size() - 1)
    {
      auto transition = task_graph_data_storage->getData(transition_keys[i].second)
                            .as<tesseract::command_language::CompositeInstruction>();
      transition.erase(transition.begin());
      program.emplace_back(transition);
    }
  }

  setData(context, INOUT_PROGRAM_PORT, program);

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

void RasterOnlyMotionTask::checkTaskInput(const tesseract::common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("RasterOnlyMotionTask, input is null");

  if (input.getType() != std::type_index(typeid(tesseract::command_language::CompositeInstruction)))
    throw std::runtime_error("RasterOnlyMotionTask, input is not a composite instruction");

  const auto& composite = input.as<tesseract::command_language::CompositeInstruction>();

  // Check rasters and transitions
  for (const auto& i : composite)
  {
    // Both rasters and transitions should be a composite
    if (!i.isCompositeInstruction())
      throw std::runtime_error("RasterOnlyMotionTask, Both rasters and transitions should be a composite");
  }
}

}  // namespace tesseract::task_composer

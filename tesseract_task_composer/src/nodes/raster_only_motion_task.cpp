/**
 * @file raster_only_motion_task.h
 * @brief Plans raster paths only
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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

#include <tesseract_task_composer/nodes/raster_only_motion_task.h>
#include <tesseract_task_composer/nodes/start_task.h>
#include <tesseract_task_composer/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/nodes/update_end_state_task.h>
#include <tesseract_task_composer/nodes/update_start_state_task.h>

#include <tesseract_task_composer/task_composer_future.h>
#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_task_composer/task_composer_plugin_factory.h>

#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_common/timer.h>

namespace
{
tesseract_planning::RasterOnlyMotionTask::TaskFactoryResults
createTask(const std::string& name,
           const std::string& task_name,
           const std::map<std::string, std::string>& input_remapping,
           const std::map<std::string, std::string>& output_remapping,
           const std::vector<std::string>& input_indexing,
           const std::vector<std::string>& output_indexing,
           const tesseract_planning::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  tesseract_planning::RasterOnlyMotionTask::TaskFactoryResults tf_results;
  tf_results.node = plugin_factory.createTaskComposerNode(task_name);
  tf_results.node->setName(name);

  if (!input_remapping.empty())
    tf_results.node->renameInputKeys(input_remapping);

  if (!output_remapping.empty())
    tf_results.node->renameOutputKeys(output_remapping);

  if (!input_indexing.empty())
  {
    std::map<std::string, std::string> input_renaming;
    for (const auto& x : input_indexing)
      input_renaming[x] = x + std::to_string(index);

    tf_results.node->renameInputKeys(input_renaming);
  }

  if (!output_indexing.empty())
  {
    std::map<std::string, std::string> output_renaming;
    for (const auto& x : output_indexing)
      output_renaming[x] = x + std::to_string(index);

    tf_results.node->renameOutputKeys(output_renaming);
  }

  tf_results.input_key = tf_results.node->getInputKeys().front();
  tf_results.output_key = tf_results.node->getOutputKeys().front();

  return tf_results;
}
}  // namespace

namespace tesseract_planning
{
RasterOnlyMotionTask::RasterOnlyMotionTask() : TaskComposerTask("RasterOnlyMotionTask", true) {}
RasterOnlyMotionTask::RasterOnlyMotionTask(std::string name,
                                           std::string input_key,
                                           std::string output_key,
                                           bool is_conditional,
                                           TaskFactory raster_task_factory,
                                           TaskFactory transition_task_factory)
  : TaskComposerTask(std::move(name), is_conditional)
  , raster_task_factory_(std::move(raster_task_factory))
  , transition_task_factory_(std::move(transition_task_factory))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

RasterOnlyMotionTask::RasterOnlyMotionTask(std::string name,
                                           const YAML::Node& config,
                                           const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("RasterOnlyMotionTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("RasterOnlyMotionTask, config 'inputs' entry currently only supports one input key");

  if (output_keys_.empty())
    throw std::runtime_error("RasterOnlyMotionTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("RasterOnlyMotionTask, config 'outputs' entry currently only supports one output key");

  if (YAML::Node raster_config = config["raster"])
  {
    std::string task_name;
    std::vector<std::string> input_indexing;
    std::vector<std::string> output_indexing;
    std::map<std::string, std::string> input_remapping;
    std::map<std::string, std::string> output_remapping;

    if (YAML::Node n = raster_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'raster' missing 'task' entry");

    if (YAML::Node n = raster_config["input_remapping"])
      input_remapping = n.as<std::map<std::string, std::string>>();

    if (YAML::Node n = raster_config["output_remapping"])
      output_remapping = n.as<std::map<std::string, std::string>>();

    if (YAML::Node n = raster_config["input_indexing"])
      input_indexing = n.as<std::vector<std::string>>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'raster' missing 'input_indexing' entry");

    if (YAML::Node n = raster_config["output_indexing"])
      output_indexing = n.as<std::vector<std::string>>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'raster' missing 'output_indexing' entry");

    raster_task_factory_ = [task_name,
                            input_remapping,
                            output_remapping,
                            input_indexing,
                            output_indexing,
                            &plugin_factory](const std::string& name, std::size_t index) {
      return createTask(
          name, task_name, input_remapping, output_remapping, input_indexing, output_indexing, plugin_factory, index);
    };
  }
  else
  {
    throw std::runtime_error("RasterOnlyMotionTask: missing 'raster' entry");
  }

  if (YAML::Node transition_config = config["transition"])
  {
    std::string task_name;
    std::vector<std::string> input_indexing;
    std::vector<std::string> output_indexing;
    std::map<std::string, std::string> input_remapping;
    std::map<std::string, std::string> output_remapping;

    if (YAML::Node n = transition_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'transition' missing 'task' entry");

    if (YAML::Node n = transition_config["input_remapping"])
      input_remapping = n.as<std::map<std::string, std::string>>();

    if (YAML::Node n = transition_config["output_remapping"])
      output_remapping = n.as<std::map<std::string, std::string>>();

    if (YAML::Node n = transition_config["input_indexing"])
      input_indexing = n.as<std::vector<std::string>>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'transition' missing 'input_indexing' entry");

    if (YAML::Node n = transition_config["output_indexing"])
      output_indexing = n.as<std::vector<std::string>>();
    else
      throw std::runtime_error("RasterOnlyMotionTask, entry 'transition' missing 'output_indexing' entry");

    transition_task_factory_ = [task_name,
                                input_remapping,
                                output_remapping,
                                input_indexing,
                                output_indexing,
                                &plugin_factory](const std::string& name, std::size_t index) {
      return createTask(
          name, task_name, input_remapping, output_remapping, input_indexing, output_indexing, plugin_factory, index);
    };
  }
  else
  {
    throw std::runtime_error("RasterOnlyMotionTask: missing 'transition' entry");
  }
}

bool RasterOnlyMotionTask::operator==(const RasterOnlyMotionTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool RasterOnlyMotionTask::operator!=(const RasterOnlyMotionTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterOnlyMotionTask::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

TaskComposerNodeInfo::UPtr RasterOnlyMotionTask::runImpl(TaskComposerInput& input,
                                                         OptionalTaskComposerExecutor executor) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->env = input.problem.env;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  tesseract_common::Timer timer;
  timer.start();

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage.getData(input_keys_[0]);
  try
  {
    checkTaskInput(input_data_poly);
  }
  catch (const std::exception& e)
  {
    info->message = e.what();
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& program = input_data_poly.template as<CompositeInstruction>();
  TaskComposerGraph task_graph;

  tesseract_common::ManipulatorInfo program_manip_info =
      program.getManipulatorInfo().getCombined(input.problem.manip_info);

  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> raster_tasks;
  raster_tasks.reserve(program.size());

  // Generate all of the raster tasks. They don't depend on anything
  std::size_t raster_idx = 0;
  for (std::size_t idx = 0; idx < program.size(); idx += 2)
  {
    // Get Raster program
    auto raster_input = program[idx].template as<CompositeInstruction>();

    // Set the manipulator info
    raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));

    // Insert Start Plan Instruction
    if (idx > 0)
    {
      const InstructionPoly& pre_input_instruction = program[idx - 1];
      assert(pre_input_instruction.isCompositeInstruction());
      const auto& tci = pre_input_instruction.as<CompositeInstruction>();
      const auto* li = tci.getLastMoveInstruction();
      assert(li != nullptr);
      raster_input.insertMoveInstruction(raster_input.begin(), *li);
    }

    const std::string task_name = "Raster #" + std::to_string(raster_idx + 1) + ": " + raster_input.getDescription();
    auto raster_results = raster_task_factory_(task_name, raster_idx + 1);
    auto raster_uuid = task_graph.addNode(std::move(raster_results.node));
    raster_tasks.emplace_back(raster_uuid, std::make_pair(raster_results.input_key, raster_results.output_key));
    input.data_storage.setData(raster_results.input_key, raster_input);

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
    auto transition_input = program[idx].template as<CompositeInstruction>();

    // Set the manipulator info
    transition_input.setManipulatorInfo(transition_input.getManipulatorInfo().getCombined(program_manip_info));

    const InstructionPoly& pre_input_instruction = program[idx - 1];
    assert(pre_input_instruction.isCompositeInstruction());
    const auto& tci = pre_input_instruction.as<CompositeInstruction>();
    const auto* li = tci.getLastMoveInstruction();
    assert(li != nullptr);
    transition_input.insertMoveInstruction(transition_input.begin(), *li);

    const std::string task_name =
        "Transition #" + std::to_string(transition_idx + 1) + ": " + transition_input.getDescription();
    auto transition_results = transition_task_factory_(task_name, transition_idx + 1);
    auto transition_uuid = task_graph.addNode(std::move(transition_results.node));
    transition_keys.emplace_back(std::make_pair(transition_results.input_key, transition_results.output_key));

    const auto& prev = raster_tasks[transition_idx];
    const auto& next = raster_tasks[transition_idx + 1];
    const auto& prev_output = prev.second.second;
    const auto& next_output = next.second.second;
    auto transition_mux_task = std::make_unique<UpdateStartAndEndStateTask>(
        "UpdateStartAndEndStateTask", prev_output, next_output, transition_results.input_key, false);
    std::string transition_mux_key = transition_mux_task->getUUIDString();
    auto transition_mux_uuid = task_graph.addNode(std::move(transition_mux_task));

    input.data_storage.setData(transition_mux_key, transition_input);

    task_graph.addEdges(transition_mux_uuid, { transition_uuid });
    task_graph.addEdges(prev.first, { transition_mux_uuid });
    task_graph.addEdges(next.first, { transition_mux_uuid });

    transition_idx++;
  }

  // Debug remove
  std::ofstream tc_out_data;
  tc_out_data.open(tesseract_common::getTempPath() + "task_composer_raster_subgraph_example.dot");
  task_graph.dump(tc_out_data);  // dump the graph including dynamic tasks
  tc_out_data.close();

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, input);
  future->wait();

  if (input.isAborted())
  {
    info->message = "Raster only subgraph failed";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  program.clear();
  for (std::size_t i = 0; i < raster_tasks.size(); ++i)
  {
    CompositeInstruction segment = input.data_storage.getData(raster_tasks[i].second.second).as<CompositeInstruction>();
    if (i != 0)
      segment.erase(segment.begin());

    program.emplace_back(segment);

    if (i < raster_tasks.size() - 1)
    {
      CompositeInstruction transition =
          input.data_storage.getData(transition_keys[i].second).as<CompositeInstruction>();
      transition.erase(transition.begin());
      program.emplace_back(transition);
    }
  }

  input.data_storage.setData(output_keys_[0], program);

  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

void RasterOnlyMotionTask::checkTaskInput(const tesseract_common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("RasterOnlyMotionTask, input is null");

  if (input.getType() != std::type_index(typeid(CompositeInstruction)))
    throw std::runtime_error("RasterOnlyMotionTask, input is not a composite instruction");

  const auto& composite = input.as<CompositeInstruction>();

  // Check rasters and transitions
  for (const auto& i : composite)
  {
    // Both rasters and transitions should be a composite
    if (!i.isCompositeInstruction())
      throw std::runtime_error("RasterOnlyMotionTask, Both rasters and transitions should be a composite");
  }
}

}  // namespace tesseract_planning

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

#include <tesseract_task_composer/planning/nodes/raster_motion_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_state_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task_info.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_command_language/composite_instruction.h>

namespace
{
tesseract_planning::RasterMotionTask::TaskFactoryResults
createTask(const std::string& name,
           const std::string& task_name,
           const std::map<std::string, std::string>& remapping,
           const std::vector<std::string>& indexing,
           const tesseract_planning::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  tesseract_planning::RasterMotionTask::TaskFactoryResults tf_results;
  tf_results.node = plugin_factory.createTaskComposerNode(task_name);
  tf_results.node->setName(name);
  if (!remapping.empty())
  {
    tf_results.node->renameInputKeys(remapping);
    tf_results.node->renameOutputKeys(remapping);
  }

  if (!indexing.empty())
  {
    std::map<std::string, std::string> renaming;
    for (const auto& x : indexing)
    {
      std::string name = task_name;
      name.append("_");
      name.append(x);
      name.append(std::to_string(index));
      renaming[x] = name;
    }

    tf_results.node->renameInputKeys(renaming);
    tf_results.node->renameOutputKeys(renaming);
  }

  tf_results.input_key = tf_results.node->getInputKeys().front();
  tf_results.output_key = tf_results.node->getOutputKeys().front();

  return tf_results;
}
}  // namespace

namespace tesseract_planning
{
RasterMotionTask::RasterMotionTask() : TaskComposerTask("RasterMotionTask", true) {}
RasterMotionTask::RasterMotionTask(std::string name,
                                   std::string input_key,
                                   std::string output_key,
                                   bool conditional,
                                   TaskFactory freespace_task_factory,
                                   TaskFactory raster_task_factory,
                                   TaskFactory transition_task_factory)
  : TaskComposerTask(std::move(name), conditional)
  , freespace_task_factory_(std::move(freespace_task_factory))
  , raster_task_factory_(std::move(raster_task_factory))
  , transition_task_factory_(std::move(transition_task_factory))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

RasterMotionTask::RasterMotionTask(std::string name,
                                   const YAML::Node& config,
                                   const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("RasterMotionTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("RasterMotionTask, config 'inputs' entry currently only supports one input key");

  if (output_keys_.empty())
    throw std::runtime_error("RasterMotionTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("RasterMotionTask, config 'outputs' entry currently only supports one output key");

  if (YAML::Node freespace_config = config["freespace"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = freespace_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionTask, entry 'freespace' missing 'task' entry");

    if (YAML::Node task_config = freespace_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionTask, entry 'freespace' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionTask, entry 'freespace' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      freespace_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                    const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      freespace_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                                  std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'freespace' entry");
  }

  if (YAML::Node raster_config = config["raster"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = raster_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionTask, entry 'raster' missing 'task' entry");

    if (YAML::Node task_config = raster_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionTask, entry 'raster' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionTask, entry 'raster' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      raster_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                 const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      raster_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                               std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'raster' entry");
  }

  if (YAML::Node transition_config = config["transition"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = transition_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionTask, entry 'transition' missing 'task' entry");

    if (YAML::Node task_config = transition_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionTask, entry 'transition' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionTask, entry 'transition' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      transition_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                     const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      transition_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                                   std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionTask: missing 'transition' entry");
  }
}

bool RasterMotionTask::operator==(const RasterMotionTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool RasterMotionTask::operator!=(const RasterMotionTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterMotionTask::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

TaskComposerNodeInfo::UPtr RasterMotionTask::runImpl(TaskComposerContext& context,
                                                     OptionalTaskComposerExecutor executor) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<MotionPlannerTaskInfo>(*this);
  info->return_value = 0;
  info->env = problem.env;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  try
  {
    checkTaskInput(input_data_poly);
  }
  catch (const std::exception& e)
  {
    info->message = e.what();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& program = input_data_poly.template as<CompositeInstruction>();
  TaskComposerGraph task_graph;

  tesseract_common::ManipulatorInfo program_manip_info = program.getManipulatorInfo().getCombined(problem.manip_info);

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
    raster_input.insertMoveInstruction(raster_input.begin(), *li);

    const std::string task_name = "Raster #" + std::to_string(raster_idx + 1) + ": " + raster_input.getDescription();
    auto raster_results = raster_task_factory_(task_name, raster_idx + 1);
    raster_results.node->setConditional(false);
    auto raster_uuid = task_graph.addNode(std::move(raster_results.node));
    raster_tasks.emplace_back(raster_uuid, std::make_pair(raster_results.input_key, raster_results.output_key));
    context.data_storage->setData(raster_results.input_key, raster_input);

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
    transition_input.insertMoveInstruction(transition_input.begin(), *li);

    const std::string task_name =
        "Transition #" + std::to_string(transition_idx + 1) + ": " + transition_input.getDescription();
    auto transition_results = transition_task_factory_(task_name, transition_idx + 1);
    transition_results.node->setConditional(false);
    auto transition_uuid = task_graph.addNode(std::move(transition_results.node));
    transition_keys.emplace_back(std::make_pair(transition_results.input_key, transition_results.output_key));

    const auto& prev = raster_tasks[transition_idx];
    const auto& next = raster_tasks[transition_idx + 1];
    const auto& prev_output = prev.second.second;
    const auto& next_output = next.second.second;
    auto transition_mux_task = std::make_unique<UpdateStartAndEndStateTask>("UpdateStartAndEndStateTask",
                                                                            transition_results.input_key,
                                                                            prev_output,
                                                                            next_output,
                                                                            transition_results.output_key,
                                                                            false);
    auto transition_mux_uuid = task_graph.addNode(std::move(transition_mux_task));

    context.data_storage->setData(transition_results.input_key, transition_input);

    task_graph.addEdges(transition_mux_uuid, { transition_uuid });
    task_graph.addEdges(prev.first, { transition_mux_uuid });
    task_graph.addEdges(next.first, { transition_mux_uuid });

    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  auto from_start_input = program[0].template as<CompositeInstruction>();
  from_start_input.setManipulatorInfo(from_start_input.getManipulatorInfo().getCombined(program_manip_info));

  auto from_start_results = freespace_task_factory_("From Start: " + from_start_input.getDescription(), 0);
  auto from_start_pipeline_uuid = task_graph.addNode(std::move(from_start_results.node));

  const auto& first_raster_output_key = raster_tasks[0].second.second;
  auto update_end_state_task = std::make_unique<UpdateEndStateTask>("UpdateEndStateTask",
                                                                    from_start_results.input_key,
                                                                    first_raster_output_key,
                                                                    from_start_results.output_key,
                                                                    false);
  auto update_end_state_uuid = task_graph.addNode(std::move(update_end_state_task));

  context.data_storage->setData(from_start_results.input_key, from_start_input);

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
  to_end_input.insertMoveInstruction(to_end_input.begin(), *li);

  auto to_end_results = freespace_task_factory_("To End: " + to_end_input.getDescription(), program.size());
  auto to_end_pipeline_uuid = task_graph.addNode(std::move(to_end_results.node));

  const auto& last_raster_output_key = raster_tasks.back().second.second;
  auto update_start_state_task = std::make_unique<UpdateStartStateTask>(
      "UpdateStartStateTask", to_end_results.input_key, last_raster_output_key, to_end_results.output_key, false);
  auto update_start_state_uuid = task_graph.addNode(std::move(update_start_state_task));

  context.data_storage->setData(to_end_results.input_key, to_end_input);

  task_graph.addEdges(update_start_state_uuid, { to_end_pipeline_uuid });
  task_graph.addEdges(raster_tasks.back().first, { update_start_state_uuid });

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, context.problem, context.data_storage);
  future->wait();

  // Merge child context data into parent context
  context.task_infos.mergeInfoMap(std::move(future->context->task_infos));
  if (future->context->isAborted())
    context.abort(future->context->task_infos.getAbortingNode());

  auto info_map = context.task_infos.getInfoMap();
  if (context.problem->dotgraph)
  {
    std::stringstream dot_graph;
    dot_graph << "subgraph cluster_" << toString(uuid_) << " {\n color=black;\n label = \"" << name_ << "\\n("
              << uuid_str_ << ")\";\n";
    task_graph.dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info->dotgraph = dot_graph.str();
  }

  if (context.isAborted())
  {
    info->message = "Raster subgraph failed";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  program.clear();
  program.emplace_back(context.data_storage->getData(from_start_results.output_key).as<CompositeInstruction>());
  for (std::size_t i = 0; i < raster_tasks.size(); ++i)
  {
    const auto& raster_output_key = raster_tasks[i].second.second;
    CompositeInstruction segment = context.data_storage->getData(raster_output_key).as<CompositeInstruction>();
    segment.erase(segment.begin());
    program.emplace_back(segment);

    if (i < raster_tasks.size() - 1)
    {
      const auto& transition_output_key = transition_keys[i].second;
      CompositeInstruction transition = context.data_storage->getData(transition_output_key).as<CompositeInstruction>();
      transition.erase(transition.begin());
      program.emplace_back(transition);
    }
  }
  CompositeInstruction to_end = context.data_storage->getData(to_end_results.output_key).as<CompositeInstruction>();
  to_end.erase(to_end.begin());
  program.emplace_back(to_end);

  context.data_storage->setData(output_keys_[0], program);

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
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

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterMotionTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterMotionTask)

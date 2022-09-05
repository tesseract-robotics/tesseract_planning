/**
 * @file raster_ft_motion_task.cpp
 * @brief Raster motion planning task with freespace transitions
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
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/task_composer_future.h>
#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_task_composer/nodes/raster_ft_motion_task.h>
#include <tesseract_task_composer/nodes/start_task.h>
#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/freespace_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/transition_mux_task.h>
#include <tesseract_task_composer/nodes/update_end_state_task.h>
#include <tesseract_task_composer/nodes/update_start_state_task.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
RasterFtMotionTask::RasterFtMotionTask(std::string input_key,
                                       std::string output_key,
                                       bool is_conditional,
                                       std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr RasterFtMotionTask::runImpl(TaskComposerInput& input,
                                                       OptionalTaskComposerExecutor executor) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(uuid_, name_);
  info->return_value = 0;

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
  auto input_data_poly = input.data_storage->getData(input_keys_[0]);
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

  auto& program = input_data_poly.as<CompositeInstruction>();
  TaskComposerGraph task_graph;

  tesseract_common::ManipulatorInfo program_manip_info = program.getManipulatorInfo().getCombined(input.manip_info);

  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  std::vector<std::pair<boost::uuids::uuid, std::string>> raster_tasks;
  raster_tasks.reserve(program.size());

  // Generate all of the raster tasks. They don't depend on anything
  std::size_t raster_idx = 0;
  for (std::size_t idx = 1; idx < program.size() - 1; idx += 2)
  {
    // Get Raster program
    auto raster_input = program[idx].as<CompositeInstruction>();

    // Set the manipulator info
    raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));

    // Get Start Plan Instruction
    MoveInstructionPoly start_instruction;
    if (idx == 1)
    {
      const InstructionPoly& from_start_input_instruction = program[0];
      assert(from_start_input_instruction.isCompositeInstruction());
      const auto& ci = from_start_input_instruction.as<CompositeInstruction>();
      const auto* li = ci.getLastMoveInstruction();
      assert(li != nullptr);
      start_instruction = *li;
    }
    else
    {
      const InstructionPoly& pre_input_instruction = program[idx - 1];
      assert(pre_input_instruction.isCompositeInstruction());
      const auto& tci = pre_input_instruction.as<CompositeInstruction>();
      const auto* li = tci.getLastMoveInstruction();
      assert(li != nullptr);
      start_instruction = *li;
    }

    // Update start state
    start_instruction.setMoveType(MoveInstructionType::START);
    raster_input.setStartInstruction(start_instruction);

    auto raster_pipeline_task = std::make_unique<CartesianMotionPipelineTask>(
        "Raster #" + std::to_string(raster_idx + 1) + ": " + raster_input.getDescription());
    std::string raster_pipeline_key = raster_pipeline_task->getUUIDString();
    auto raster_pipeline_uuid = task_graph.addNode(std::move(raster_pipeline_task));
    raster_tasks.emplace_back(raster_pipeline_uuid, raster_pipeline_key);
    input.data_storage->setData(raster_pipeline_key, raster_input);

    task_graph.addEdges(start_uuid, { raster_pipeline_uuid });

    raster_idx++;
  }

  // Loop over all transitions
  std::vector<std::string> transition_keys;
  transition_keys.reserve(program.size());
  std::size_t transition_idx = 0;
  for (std::size_t idx = 2; idx < program.size() - 2; idx += 2)
  {
    // Get transition program
    auto transition_input = program[idx].as<CompositeInstruction>();

    // Set the manipulator info
    transition_input.setManipulatorInfo(transition_input.getManipulatorInfo().getCombined(program_manip_info));

    auto transition_pipeline_task = std::make_unique<FreespaceMotionPipelineTask>(
        "Transition #" + std::to_string(transition_idx + 1) + ": " + transition_input.getDescription());
    std::string transition_pipeline_key = transition_pipeline_task->getUUIDString();
    auto transition_pipeline_uuid = task_graph.addNode(std::move(transition_pipeline_task));
    transition_keys.push_back(transition_pipeline_key);

    const auto& prev = raster_tasks[transition_idx];
    const auto& next = raster_tasks[transition_idx + 1];
    auto transition_mux_task =
        std::make_unique<TransitionMuxTask>(prev.second, next.second, transition_pipeline_key, false);
    std::string transition_mux_key = transition_mux_task->getUUIDString();
    auto transition_mux_uuid = task_graph.addNode(std::move(transition_mux_task));

    input.data_storage->setData(transition_mux_key, transition_input);

    task_graph.addEdges(transition_mux_uuid, { transition_pipeline_uuid });
    task_graph.addEdges(prev.first, { transition_mux_uuid });
    task_graph.addEdges(next.first, { transition_mux_uuid });

    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  auto from_start_input = program[0].as<CompositeInstruction>();

  from_start_input.setStartInstruction(program.getStartInstruction());
  from_start_input.setManipulatorInfo(from_start_input.getManipulatorInfo().getCombined(program_manip_info));

  auto from_start_pipeline_task =
      std::make_unique<FreespaceMotionPipelineTask>("From Start: " + from_start_input.getDescription());
  std::string from_start_pipeline_key = from_start_pipeline_task->getUUIDString();
  auto from_start_pipeline_uuid = task_graph.addNode(std::move(from_start_pipeline_task));

  auto update_end_state_task =
      std::make_unique<UpdateEndStateTask>(raster_tasks[0].second, from_start_pipeline_key, false);
  std::string update_end_state_key = update_end_state_task->getUUIDString();
  auto update_end_state_uuid = task_graph.addNode(std::move(update_end_state_task));

  input.data_storage->setData(update_end_state_key, from_start_input);

  task_graph.addEdges(update_end_state_uuid, { from_start_pipeline_uuid });
  task_graph.addEdges(raster_tasks[0].first, { update_end_state_uuid });

  // Plan to_end - preceded by the last raster
  auto to_end_input = program.back().as<CompositeInstruction>();

  to_end_input.setManipulatorInfo(to_end_input.getManipulatorInfo().getCombined(program_manip_info));

  auto to_end_pipeline_task = std::make_unique<FreespaceMotionPipelineTask>("To End: " + to_end_input.getDescription());
  std::string to_end_pipeline_key = to_end_pipeline_task->getUUIDString();
  auto to_end_pipeline_uuid = task_graph.addNode(std::move(to_end_pipeline_task));

  auto update_start_state_task =
      std::make_unique<UpdateStartStateTask>(raster_tasks.back().second, to_end_pipeline_key, false);
  std::string update_start_state_key = update_start_state_task->getUUIDString();
  auto update_start_state_uuid = task_graph.addNode(std::move(update_start_state_task));

  input.data_storage->setData(update_start_state_key, to_end_input);

  task_graph.addEdges(update_start_state_uuid, { to_end_pipeline_uuid });
  task_graph.addEdges(raster_tasks.back().first, { update_start_state_uuid });

  // Debug remove
  std::ofstream tc_out_data;
  tc_out_data.open(tesseract_common::getTempPath() + "task_composer_raster_subgraph_example.dot");
  task_graph.dump(tc_out_data);  // dump the graph including dynamic tasks
  tc_out_data.close();

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, input);
  future->wait();

  if (input.isAborted())
  {
    info->message = "Raster subgraph failed";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  program.clear();
  program.appendInstruction(input.data_storage->getData(from_start_pipeline_key).as<CompositeInstruction>());
  for (std::size_t i = 0; i < raster_tasks.size(); ++i)
  {
    program.appendInstruction(input.data_storage->getData(raster_tasks[i].second).as<CompositeInstruction>());
    if (i < raster_tasks.size() - 1)
      program.appendInstruction(input.data_storage->getData(transition_keys[i]).as<CompositeInstruction>());
  }
  program.appendInstruction(input.data_storage->getData(to_end_pipeline_key).as<CompositeInstruction>());

  input.data_storage->setData(output_keys_[0], program);

  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

void RasterFtMotionTask::checkTaskInput(const tesseract_common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("RasterFtMotionTask, input is null");

  if (input.getType() != std::type_index(typeid(CompositeInstruction)))
    throw std::runtime_error("RasterFtMotionTask, input is not a composite instruction");

  const auto& composite = input.as<CompositeInstruction>();

  // Check that it has a start instruction
  if (!composite.hasStartInstruction())
    throw std::runtime_error("RasterFtMotionTask, input should have a start instruction");

  // Check from_start
  if (!composite.at(0).isCompositeInstruction())
    throw std::runtime_error("RasterFtMotionTask, from_start should be a composite");

  // Check rasters and transitions
  for (std::size_t index = 1; index < composite.size() - 1; index++)
  {
    // Both rasters and transitions should be a composite
    if (!composite.at(index).isCompositeInstruction())
      throw std::runtime_error("RasterFtMotionTask, Both rasters and transitions should be a composite");
  }

  // Check to_end
  if (!composite.back().isCompositeInstruction())
    throw std::runtime_error("RasterFtMotionTask, to_end should be a composite");
}

bool RasterFtMotionTask::operator==(const RasterFtMotionTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool RasterFtMotionTask::operator!=(const RasterFtMotionTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterFtMotionTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterFtMotionTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterFtMotionTask)

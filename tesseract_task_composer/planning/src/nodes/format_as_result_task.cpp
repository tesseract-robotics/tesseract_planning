#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/format_as_result_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

namespace tesseract::task_composer
{
// Requried
const std::string FormatAsResultTask::INOUT_PROGRAMS_PORT = "programs";

FormatAsResultTask::FormatAsResultTask() : TaskComposerTask("FormatAsResultTask", FormatAsResultTask::ports(), true) {}

FormatAsResultTask::FormatAsResultTask(std::string name,
                                       const std::vector<std::string>& input_keys,
                                       const std::vector<std::string>& output_keys,
                                       bool is_conditional)
  : TaskComposerTask(std::move(name), FormatAsResultTask::ports(), is_conditional)
{
  input_keys_.add(INOUT_PROGRAMS_PORT, input_keys);
  output_keys_.add(INOUT_PROGRAMS_PORT, output_keys);

  validatePorts();

  if (input_keys_.size() != output_keys_.size())
    throw std::runtime_error("FormatAsResultTask, input_keys and output_keys be the same size!");
}

FormatAsResultTask::FormatAsResultTask(std::string name,
                                       const YAML::Node& config,
                                       const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), FormatAsResultTask::ports(), config)
{
  if (input_keys_.size() != output_keys_.size())
    throw std::runtime_error("FormatAsResultTask, input_keys and output_keys be the same size!");
}

TaskComposerNodePorts FormatAsResultTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAMS_PORT] = TaskComposerNodePorts::MULTIPLE;
  ports.output_required[INOUT_PROGRAMS_PORT] = TaskComposerNodePorts::MULTIPLE;
  return ports;
}

TaskComposerNodeInfo FormatAsResultTask::runImpl(TaskComposerContext& context,
                                                 OptionalTaskComposerExecutor /*executor*/) const
{
  auto input_data_container = getData<std::vector<tesseract::common::AnyPoly>>(context, INOUT_PROGRAMS_PORT);
  std::vector<tesseract::common::AnyPoly> output_data_container;
  output_data_container.reserve(input_data_container.size());
  for (auto& input_data : input_data_container)
  {
    auto& ci = input_data.as<tesseract::command_language::CompositeInstruction>();
    std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>> instructions =
        ci.flatten(&tesseract::command_language::moveFilter);
    for (auto& instruction : instructions)
    {
      auto& mi = instruction.get().as<tesseract::command_language::MoveInstructionPoly>();
      if (mi.getWaypoint().isStateWaypoint())
        continue;

      if (!mi.getWaypoint().isJointWaypoint())
        throw std::runtime_error("FormatAsResultTask, unsupported waypoint type!");

      auto& jwp = mi.getWaypoint().as<tesseract::command_language::JointWaypointPoly>();

      // Convert to StateWaypoint
      tesseract::command_language::StateWaypointPoly swp = mi.createStateWaypoint();
      swp.setName(jwp.getName());
      swp.setNames(jwp.getNames());
      swp.setPosition(jwp.getPosition());
      mi.getWaypoint() = swp;
    }

    output_data_container.emplace_back(ci);
  }

  setData(context, INOUT_PROGRAMS_PORT, output_data_container);

  TaskComposerNodeInfo info(*this);
  info.color = "green";
  info.return_value = 1;
  info.status_code = 1;
  info.status_message = "Successful";
  return info;
}

}  // namespace tesseract::task_composer

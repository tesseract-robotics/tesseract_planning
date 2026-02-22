#ifndef TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H
#define TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT FormatAsResultTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAMS_PORT;

  using Ptr = std::shared_ptr<FormatAsResultTask>;
  using ConstPtr = std::shared_ptr<const FormatAsResultTask>;
  using UPtr = std::unique_ptr<FormatAsResultTask>;
  using ConstUPtr = std::unique_ptr<const FormatAsResultTask>;

  FormatAsResultTask();
  explicit FormatAsResultTask(std::string name,
                              const std::vector<std::string>& input_keys,
                              const std::vector<std::string>& output_keys,
                              bool is_conditional = true);
  explicit FormatAsResultTask(std::string name,
                              const YAML::Node& config,
                              const TaskComposerPluginFactory& plugin_factory);
  ~FormatAsResultTask() override = default;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H

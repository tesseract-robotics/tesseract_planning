#ifndef TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H
#define TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/core/tesseract_task_composer_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;
class TESSERACT_TASK_COMPOSER_NODES_EXPORT HasDataStorageEntryTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_KEYS_PORT;

  using Ptr = std::shared_ptr<HasDataStorageEntryTask>;
  using ConstPtr = std::shared_ptr<const HasDataStorageEntryTask>;
  using UPtr = std::unique_ptr<HasDataStorageEntryTask>;
  using ConstUPtr = std::unique_ptr<const HasDataStorageEntryTask>;

  HasDataStorageEntryTask();
  explicit HasDataStorageEntryTask(std::string name,
                                   const std::vector<std::string>& input_keys,
                                   bool is_conditional = true);
  explicit HasDataStorageEntryTask(std::string name,
                                   const YAML::Node& config,
                                   const TaskComposerPluginFactory& plugin_factory);
  ~HasDataStorageEntryTask() override = default;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/has_data_storage_entry_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract_planning
{
const std::string HasDataStorageEntryTask::INPUT_KEYS_PORT = "keys";

HasDataStorageEntryTask::HasDataStorageEntryTask()
  : TaskComposerTask("HasDataStorageEntryTask", HasDataStorageEntryTask::ports(), true)
{
}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const std::vector<std::string>& input_keys,
                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), HasDataStorageEntryTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_KEYS_PORT, input_keys);
  validatePorts();
}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const YAML::Node& config,
                                                 const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), HasDataStorageEntryTask::ports(), config)
{
}

TaskComposerNodePorts HasDataStorageEntryTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_KEYS_PORT] = TaskComposerNodePorts::MULTIPLE;
  return ports;
}

TaskComposerNodeInfo HasDataStorageEntryTask::runImpl(TaskComposerContext& context,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);

  // Get local data storage
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);

  const auto& keys = input_keys_.get<std::vector<std::string>>(INPUT_KEYS_PORT);
  for (const auto& key : keys)
  {
    if (!data_storage->hasKey(key))
    {
      info.color = "red";
      info.return_value = 0;
      info.status_code = 0;
      info.status_message = "Missing input key: " + key;
      return info;
    }
  }

  info.color = "green";
  info.return_value = 1;
  info.status_code = 1;
  info.status_message = "Successful";
  return info;
}

}  // namespace tesseract_planning

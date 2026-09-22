#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/nodes/has_data_storage_entry_task.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_node_info.h>

namespace tesseract::task_composer
{

HasDataStorageEntryTask::HasDataStorageEntryTask()
  : TaskComposerTask("HasDataStorageEntryTask", HasDataStorageEntryTask::ports(), true)
{
}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const std::vector<std::string>& input_storage_keys,
                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), HasDataStorageEntryTask::ports(), is_conditional)
{
  input_port_mappings_.set(INPUT_STORAGE_KEYS_PORT, input_storage_keys);
  setPortMappings(input_port_mappings_, output_port_mappings_);
}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const YAML::Node& config,
                                                 const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), HasDataStorageEntryTask::ports(), config)
{
}

tesseract::common::PropertyTree HasDataStorageEntryTask::schema() { return TaskComposerTask::schema(ports()); }

const TaskComposerNodePorts& HasDataStorageEntryTask::ports()
{
  static const TaskComposerNodePorts ports = []() {
    TaskComposerNodePorts ports;
    ports.addRequiredInput(INPUT_STORAGE_KEYS_PORT, TaskComposerNodePorts::Cardinality::MULTIPLE);
    return ports;
  }();
  return ports;
}

TaskComposerNodeInfo HasDataStorageEntryTask::runImpl(TaskComposerContext& context,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);

  // Get local data storage
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);

  const auto& storage_keys = input_port_mappings_.multiple(INPUT_STORAGE_KEYS_PORT);
  for (const auto& storage_key : storage_keys)
  {
    if (!data_storage->hasKey(storage_key))
    {
      info.color = "red";
      info.return_value = 0;
      info.status_code = 0;
      info.status_message = "Missing input storage key: " + storage_key;
      return info;
    }
  }

  info.color = "green";
  info.return_value = 1;
  info.status_code = 1;
  info.status_message = "Successful";
  return info;
}

}  // namespace tesseract::task_composer

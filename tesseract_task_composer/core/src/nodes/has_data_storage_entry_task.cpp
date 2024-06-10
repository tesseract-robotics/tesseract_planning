#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/map.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/has_data_storage_entry_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract_planning
{
HasDataStorageEntryTask::HasDataStorageEntryTask() : TaskComposerTask("HasDataStorageEntryTask", true) {}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const std::vector<std::string>& input_keys,
                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_ = input_keys;
  if (input_keys_.empty())
    throw std::runtime_error("HasDataStorageEntryTask, input_keys should not be empty!");
}
HasDataStorageEntryTask::HasDataStorageEntryTask(std::string name,
                                                 const YAML::Node& config,
                                                 const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("HasDataStorageEntryTask, input_keys should not be empty!");

  if (!output_keys_.empty())
    throw std::runtime_error("HasDataStorageEntryTask, output_keys should be empty!");
}

std::unique_ptr<TaskComposerNodeInfo> HasDataStorageEntryTask::runImpl(TaskComposerContext& context,
                                                                       OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  for (const auto& input_key : input_keys_)
  {
    if (!context.data_storage->hasKey(input_key))
    {
      info->color = "red";
      info->return_value = 0;
      info->status_code = 0;
      info->status_message = "Missing input key";
      return info;
    }
  }

  info->color = "green";
  info->return_value = 1;
  info->status_code = 1;
  info->status_message = "Successful";
  return info;
}

bool HasDataStorageEntryTask::operator==(const HasDataStorageEntryTask& rhs) const
{
  return (TaskComposerNode::operator==(rhs));
}
bool HasDataStorageEntryTask::operator!=(const HasDataStorageEntryTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void HasDataStorageEntryTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::HasDataStorageEntryTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::HasDataStorageEntryTask)

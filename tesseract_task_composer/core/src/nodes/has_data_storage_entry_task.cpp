#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/map.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/property_tree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/has_data_storage_entry_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_schema.h>

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

tesseract_common::PropertyTree HasDataStorageEntryTask::getSchema() const
{
  using namespace tesseract_common;

  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::REQUIRED, true);
  schema.setAttribute(property_attribute::TASK_NAME, "HasDataStorageEntryTask");
  schema.setAttribute(property_attribute::FACTORY_NAME, "HasDataStorageEntryTaskFactory");
  schema.setAttribute(property_attribute::DOC, "A task to check for entries in the data storage");

  std::map<int, std::string> return_options;
  return_options[0] = "Error";
  return_options[0] = "Successful";
  schema.setAttribute("return_options", YAML::Node(return_options));

  addConditionalProperty(schema, false);
  addTriggerAbortProperty(schema);

  PropertyTree& inputs = addInputsProperty(schema);
  {
    auto& prop = inputs[INPUT_KEYS_PORT];
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "A list of keys to check for entries in data storage");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  return schema;
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
  const auto& keys = input_keys_.get<std::vector<std::string>>(INPUT_KEYS_PORT);
  for (const auto& key : keys)
  {
    if (!context.data_storage->hasKey(key))
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

bool HasDataStorageEntryTask::operator==(const HasDataStorageEntryTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool HasDataStorageEntryTask::operator!=(const HasDataStorageEntryTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void HasDataStorageEntryTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::HasDataStorageEntryTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::HasDataStorageEntryTask)

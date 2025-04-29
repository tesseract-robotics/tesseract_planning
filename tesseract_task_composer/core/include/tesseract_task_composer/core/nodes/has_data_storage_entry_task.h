#ifndef TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H
#define TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <tesseract_task_composer/core/tesseract_task_composer_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
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

  bool operator==(const HasDataStorageEntryTask& rhs) const;
  bool operator!=(const HasDataStorageEntryTask& rhs) const;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::HasDataStorageEntryTask)

#endif  // TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

#ifndef TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H
#define TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class HasDataStorageEntryTask : public TaskComposerTask
{
public:
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

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::unique_ptr<TaskComposerNodeInfo>
  runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY2(tesseract_planning::HasDataStorageEntryTask, "HasDataStorageEntryTask")

#endif  // TESSERACT_TASK_COMPOSER_HAS_DATA_STORAGE_ENTRY_TASK_H

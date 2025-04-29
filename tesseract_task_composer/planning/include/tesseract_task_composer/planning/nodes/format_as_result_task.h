#ifndef TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H
#define TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
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

  bool operator==(const FormatAsResultTask& rhs) const;
  bool operator!=(const FormatAsResultTask& rhs) const;

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

BOOST_CLASS_EXPORT_KEY(tesseract_planning::FormatAsResultTask)

#endif  // TESSERACT_TASK_COMPOSER_FORMAT_AS_RESULT_TASK_H

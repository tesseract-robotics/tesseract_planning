/**
 * @file remap_task.h
 *
 * @author Levi Armstrong
 * @date July 13, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_TASK_COMPOSER_REMAP_TASK_H
#define TESSERACT_TASK_COMPOSER_REMAP_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class RemapTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<RemapTask>;
  using ConstPtr = std::shared_ptr<const RemapTask>;
  using UPtr = std::unique_ptr<RemapTask>;
  using ConstUPtr = std::unique_ptr<const RemapTask>;

  // Requried
  static const std::string INOUT_KEYS_PORT;

  RemapTask();
  explicit RemapTask(std::string name,
                     const std::map<std::string, std::string>& remap,
                     bool copy = false,
                     bool is_conditional = false);
  explicit RemapTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~RemapTask() override = default;

  bool operator==(const RemapTask& rhs) const;
  bool operator!=(const RemapTask& rhs) const;

private:
  bool copy_{ false };

  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::RemapTask)
#endif  // TESSERACT_TASK_COMPOSER_REMAP_TASK_H

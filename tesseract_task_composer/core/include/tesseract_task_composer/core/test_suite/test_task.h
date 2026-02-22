/**
 * @file test_task.h
 *
 * @author Levi Armstrong
 * @date June 26, 2023
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
#ifndef TESSERACT_TASK_COMPOSER_TEST_TASK_H
#define TESSERACT_TASK_COMPOSER_TEST_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/fwd.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;
}

namespace tesseract::task_composer::test_suite
{
class DummyTaskComposerNode : public TaskComposerNode
{
  using TaskComposerNode::TaskComposerNode;

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const override final;
};

class TestTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PORT1_PORT;
  static const std::string INOUT_PORT2_PORT;

  TestTask();
  explicit TestTask(std::string name, bool conditional);
  explicit TestTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~TestTask() override = default;

  bool throw_exception{ false };
  bool set_abort{ false };
  int return_value{ 0 };

  bool operator==(const TestTask& rhs) const;
  bool operator!=(const TestTask& rhs) const;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const override final;
};
}  // namespace tesseract::task_composer::test_suite

#endif  // TESSERACT_TASK_COMPOSER_TEST_TASK_H

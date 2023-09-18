/**
 * @file test_task.h
 *
 * @author Levi Armstrong
 * @date June 26, 2023
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
#ifndef TESSERACT_TASK_COMPOSER_TEST_TASK_H
#define TESSERACT_TASK_COMPOSER_TEST_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
}

namespace tesseract_planning::test_suite
{
class TestTask : public TaskComposerTask
{
public:
  TestTask();
  explicit TestTask(std::string name, bool conditional);
  explicit TestTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~TestTask() override = default;

  bool throw_exception{ false };
  bool set_abort{ false };
  int return_value{ 0 };

  bool operator==(const TestTask& rhs) const;
  bool operator!=(const TestTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const override final;
};
}  // namespace tesseract_planning::test_suite

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::test_suite::TestTask, "TestTask")

#endif  // TESSERACT_TASK_COMPOSER_TEST_TASK_H

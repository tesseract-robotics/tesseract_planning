/**
 * @file test_task.cpp
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

#include <tesseract_task_composer/core/test_suite/test_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning::test_suite
{
TestTask::TestTask() : TaskComposerTask("TestTask", true) {}
TestTask::TestTask(std::string name, bool is_conditional) : TaskComposerTask(std::move(name), is_conditional) {}
TestTask::TestTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  // LCOV_EXCL_START
  try
  {
    if (YAML::Node n = config["throw_exception"])
      throw_exception = n.as<bool>();

    if (YAML::Node n = config["set_abort"])
      set_abort = n.as<bool>();

    if (YAML::Node n = config["return_value"])
      return_value = n.as<int>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TestTask: Failed to parse yaml config data! Details: " + std::string(e.what()));
  }
  // LCOV_EXCL_STOP
}

bool TestTask::operator==(const TestTask& rhs) const
{
  bool equal = true;
  equal &= (throw_exception == rhs.throw_exception);
  equal &= (set_abort == rhs.set_abort);
  equal &= (return_value == rhs.return_value);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool TestTask::operator!=(const TestTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void TestTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(throw_exception);
  ar& BOOST_SERIALIZATION_NVP(set_abort);
  ar& BOOST_SERIALIZATION_NVP(return_value);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

TaskComposerNodeInfo::UPtr TestTask::runImpl(TaskComposerContext& context,
                                             OptionalTaskComposerExecutor /*executor*/) const
{
  if (throw_exception)
    throw std::runtime_error("TestTask, failure");

  auto node_info = std::make_unique<TaskComposerNodeInfo>(*this);
  if (conditional_)
    node_info->color = (return_value == 0) ? "red" : "green";
  else
    node_info->color = "green";
  node_info->return_value = return_value;

  if (set_abort)
  {
    node_info->color = "red";
    context.abort(uuid_);
  }

  return node_info;
}

}  // namespace tesseract_planning::test_suite

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::test_suite::TestTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::test_suite::TestTask)

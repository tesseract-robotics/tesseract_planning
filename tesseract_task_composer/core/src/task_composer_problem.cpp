/**
 * @file task_composer_problem.cpp
 * @brief A task composer server problem
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/shared_ptr.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_problem.h>

namespace tesseract_planning
{
TaskComposerProblem::TaskComposerProblem(std::string name, bool dotgraph) : name(std::move(name)), dotgraph(dotgraph) {}

TaskComposerProblem::UPtr TaskComposerProblem::clone() const { return std::make_unique<TaskComposerProblem>(*this); }

bool TaskComposerProblem::operator==(const TaskComposerProblem& rhs) const
{
  bool equal = true;
  equal &= name == rhs.name;
  equal &= dotgraph == rhs.dotgraph;
  equal &= input == rhs.input;
  return equal;
}

bool TaskComposerProblem::operator!=(const TaskComposerProblem& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerProblem::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name);
  ar& boost::serialization::make_nvp("dotgraph", dotgraph);
  ar& boost::serialization::make_nvp("input", input);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerProblem)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerProblem)

/**
 * @copyright Copyright (c) 2024, Levi Armstrong
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

#include <tesseract_task_composer/core/task_composer_log.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_common/serialization.h>
#include <boost/serialization/shared_ptr.hpp>

namespace tesseract_planning
{
TaskComposerLog::TaskComposerLog(std::string desc) : description(std::move(desc)) {}

bool TaskComposerLog::operator==(const TaskComposerLog& rhs) const
{
  return ((description == rhs.description) && (initial_data == rhs.initial_data) && (*context == *rhs.context) &&
          (dotgraph == rhs.dotgraph));
}

// LCOV_EXCL_START
bool TaskComposerLog::operator!=(const TaskComposerLog& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TaskComposerLog::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(description);
  ar& BOOST_SERIALIZATION_NVP(initial_data);
  ar& BOOST_SERIALIZATION_NVP(context);
  ar& BOOST_SERIALIZATION_NVP(dotgraph);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerLog)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerLog)

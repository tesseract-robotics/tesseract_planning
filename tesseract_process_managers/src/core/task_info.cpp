/**
 * @file task_info.h
 * @brief Process Info
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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

#include <tesseract_process_managers/core/task_info.h>

namespace tesseract_planning
{
TaskInfo::TaskInfo(std::size_t unique_id, std::string name) : unique_id(unique_id), task_name(std::move(name)) {}

bool TaskInfo::operator==(const TaskInfo& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= return_value == rhs.return_value;
  equal &= unique_id == rhs.unique_id;
  equal &= task_name == rhs.task_name;
  equal &= message == rhs.message;
  equal &= tesseract_common::almostEqualRelativeAndAbs(elapsed_time, rhs.elapsed_time, max_diff);
  equal &= instructions_input == rhs.instructions_input;
  equal &= instructions_output == rhs.instructions_output;
  equal &= results_input == rhs.results_input;
  equal &= results_output == rhs.results_output;
  return equal;
}
bool TaskInfo::operator!=(const TaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("return_value", return_value);
  ar& boost::serialization::make_nvp("unique_id", unique_id);
  ar& boost::serialization::make_nvp("task_name", task_name);
  ar& boost::serialization::make_nvp("message", message);
  ar& boost::serialization::make_nvp("elapsed_time", elapsed_time);

  ar& boost::serialization::make_nvp("instructions_input", instructions_input);
  ar& boost::serialization::make_nvp("instructions_output", instructions_output);
  ar& boost::serialization::make_nvp("results_input", results_input);
  ar& boost::serialization::make_nvp("results_output", results_output);

  //  ar& boost::serialization::make_nvp("environment", environment);
}

void TaskInfoContainer::addTaskInfo(TaskInfo::ConstPtr task_info)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  task_info_map_[task_info->unique_id] = std::move(task_info);
}

TaskInfo::ConstPtr TaskInfoContainer::operator[](std::size_t index) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return task_info_map_.at(index);
}

std::map<std::size_t, TaskInfo::ConstPtr> TaskInfoContainer::getTaskInfoMap() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return task_info_map_;
}

}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::TaskInfo::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);
template void tesseract_planning::TaskInfo::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);

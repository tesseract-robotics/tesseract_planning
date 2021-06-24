/**
 * @file task_info_utils.cpp
 * @brief Task Info Utils
 *
 * @author Matthew Powelson
 * @date June 22. 2021
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/serialization.h>
#include <tesseract_process_managers/utils/task_info_statistics.h>

using namespace tesseract_planning;

void TaskInfoStatistics::insert(const std::vector<TaskInfo>& task_info_vec)
{
  if (task_info_vec.empty())
    return;

  if (task_name.empty() && occurances == 0)
    task_name = task_info_vec.front().task_name;

  for (const TaskInfo& task_info : task_info_vec)
  {
    if (task_name != task_info.task_name)
      CONSOLE_BRIDGE_logError("Incorrect TaskInfo passed into TaskInfoStatistics");

    // Calculate statistics
    min_time = std::min(min_time, task_info.elapsed_time);
    max_time = std::max(max_time, task_info.elapsed_time);
    const double old_sum = avg_time * static_cast<double>(occurances);
    occurances++;
    avg_time = (old_sum + task_info.elapsed_time) / static_cast<double>(occurances);

    return_val_map[task_info.return_value] += 1;
  }
}

void TaskInfoProfiler::load(const tesseract_common::fs::path& directory)
{
  // Each file will contain the TaskInfos for a planning run
  std::vector<std::vector<TaskInfo>> task_info_vecs;
  for (auto& entry : boost::make_iterator_range(tesseract_common::fs::directory_iterator(directory), {}))
  {
    const std::string filepath = entry.path().string();
    try
    {
      std::vector<TaskInfo> task_info = Serialization::fromArchiveFileXML<std::vector<TaskInfo>>(filepath);
      task_info_vecs.push_back(task_info);
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
    }
  }

  load(task_info_vecs);
}

void TaskInfoProfiler::load(const std::vector<std::vector<TaskInfo>>& task_info_vecs)
{
  // Sort into map by task names
  std::unordered_map<std::string, std::vector<TaskInfo>> sorted_infos = sortTaskInfosByTaskName(task_info_vecs);

  // Loop over TaskNames and insert into the TaskInfoStatistics
  for (const auto& kv : sorted_infos)
  {
    stats_map[kv.first].insert(kv.second);
  }
}

void TaskInfoProfiler::load(const std::map<std::size_t, TaskInfo::ConstPtr>& task_info_map)
{
  for (const auto& kv : task_info_map)
  {
    stats_map[kv.second->task_name].insert({ *kv.second });
  }
}

void TaskInfoProfiler::clear() { stats_map.clear(); }

void TaskInfoProfiler::print(std::ostream& os) const
{
  using std::setw;
  const std::vector<std::string> column_names = { "Task Name",    "Occurances",    "Min Time (s)",  "Max Time (s)",
                                                  "Avg Time (s)", "Return values", "Return value %" };
  const int column_width = 20;

  // Print column headers
  os << "|";
  for (const std::string& column_name : column_names)
  {
    // Make the name column extra long
    if (column_name == "Task Name")
      os << setw(column_width * 2) << column_name;
    else
      os << setw(column_width) << column_name;

    os << "|";
  }
  os << "\n|" << std::string((column_names.size() + 1) * (column_width + 1) - 2, '-') << "| \n";

  // Print statistics
  for (const std::pair<std::string, TaskInfoStatistics>& kv : stats_map)
  {
    os << "|";

    // Name
    os << setw(column_width * 2) << kv.first.substr(0, column_width * 2) << "|";
    // Occurances
    os << setw(column_width) << kv.second.occurances << "|";
    // Time
    os << setw(column_width) << kv.second.min_time << "|" << setw(column_width) << kv.second.max_time << "|"
       << setw(column_width) << kv.second.avg_time << "|";
    // Return value
    std::string ret_val_str;
    std::string ret_val_perc_str;
    for (const auto& ret_kv : kv.second.return_val_map)
    {
      ret_val_str += std::to_string(ret_kv.first) + ", ";
      double ret_val_percent = static_cast<double>(ret_kv.second) / static_cast<double>(kv.second.occurances) * 100.;
      ret_val_perc_str += std::to_string(ret_val_percent).substr(0, 5) + ", ";
    }
    os << setw(column_width) << ret_val_str << "|" << setw(column_width) << ret_val_perc_str << "|";
    os << "\n";
  }
}

std::unordered_map<std::string, TaskDisplayInfo> TaskInfoProfiler::getTaskDisplayInfo()
{
  std::unordered_map<std::string, TaskDisplayInfo> task_info_map;

  // Calculate and print statistics
  for (const std::pair<std::string, TaskInfoStatistics>& kv : stats_map)
  {
    if (kv.second.occurances == 0)
      continue;

    // Task Info
    task_info_map[kv.first].task_info = "Avg time: " + std::to_string(kv.second.avg_time).substr(0, 5) + "s";

    // Edge info
    for (const std::pair<int, int>& ret_kv : kv.second.return_val_map)
    {
      double percent = static_cast<double>(ret_kv.second) / static_cast<double>(kv.second.occurances) * 100.;
      task_info_map[kv.first].edge_info[ret_kv.first] = std::to_string(percent).substr(0, 5) + "%";
    }
  }
  return task_info_map;
}

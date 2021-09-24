/**
 * @file task_info_utils.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_TASK_INFO_UTILS_H
#define TESSERACT_PROCESS_MANAGERS_TASK_INFO_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/utils/taskflow_visualization_utils.h>

namespace tesseract_planning
{
/**
 * @brief Container for stats on one TaskInfo
 */
struct TaskInfoStatistics
{
  std::string task_name;
  void insert(const std::vector<TaskInfo>& task_info_vec);

  int occurances{ 0 };
  double min_time{ std::numeric_limits<double>::max() };
  double max_time{ 0 };
  double avg_time{ 0 };

  /** @brief Key: return value, Value: occurances */
  std::map<int, int> return_val_map;
};

/**
 * @brief Loads TaskInfos from various sources and calculates some statistics
 */
class TaskInfoProfiler
{
public:
  /**
   * @brief Load a directory of std::vector<TaskInfo> serialized to XML
   * @param directory Directory containing serialized TaskInfos
   */
  void load(const tesseract_common::fs::path& directory);
  /**
   * @brief Load task info vectors
   * @param task_info_vecs
   */
  void load(const std::vector<std::vector<TaskInfo>>& task_info_vecs);
  /**
   * @brief Load from format provided from TaskInfoContainer. Probably used like
   * load(response.interface->getTaskInfoMap());
   * @param task_info_map Key: unique_id (unused), Value: TaskInfo to be inserted
   */
  void load(const std::map<std::size_t, TaskInfo::ConstPtr>& task_info_map);

  /** @brief Clears any loaded data */
  void clear();

  /**
   * @brief Prints summary statistics to the stream
   * @param os Stream to which the statistics will be printed
   */
  void print(std::ostream& os = std::cout) const;

  /**
   * @brief Gets information about each task to be used in displays
   *
   * Example: dump(os, graph, profiler.getTaskDisplayInfo());
   * @return Information about each task to be used in displays
   */
  std::unordered_map<std::string, TaskDisplayInfo> getTaskDisplayInfo();

private:
  /** @brief Key: Task name, Value: Statistics */
  std::unordered_map<std::string, TaskInfoStatistics> stats_map;
};

/**
 * @brief Takes multiple vectors of task infos from repeated planning calls and sorts them into a map keyed by task name
 *
 * This is so that we can look up all task infos by the task name since they will have different unique ids for each
 * planning run
 * @param task_infos Each planning call produces a vector of TaskInfos. This is a vector of those vectors
 * @return Task infos sorted into a map keyed by task name
 */
inline std::unordered_map<std::string, std::vector<TaskInfo>>
sortTaskInfosByTaskName(const std::vector<std::vector<TaskInfo>>& task_infos)
{
  std::unordered_map<std::string, std::vector<TaskInfo>> task_info_map;
  // Loop over planning calls
  for (const auto& task_info_vector : task_infos)
  {
    // Loop over tasks in the planning Taskflow
    for (const TaskInfo& task_info : task_info_vector)
    {
      // Insert them into the map so we can look them up by task_name
      task_info_map[task_info.task_name].push_back(task_info);
    }
  }
  return task_info_map;
}
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_task_info_H

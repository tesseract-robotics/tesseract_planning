/**
 * @file time_optimal_parameterization_task.h
 * @brief Perform TOTG
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Jan 22, 2021
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
#ifndef TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H
#define TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class TimeOptimalParameterizationTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<TimeOptimalParameterizationTask>;
  using ConstPtr = std::shared_ptr<const TimeOptimalParameterizationTask>;
  using UPtr = std::unique_ptr<TimeOptimalParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const TimeOptimalParameterizationTask>;

  TimeOptimalParameterizationTask();
  explicit TimeOptimalParameterizationTask(std::string name,
                                           std::string input_key,
                                           std::string output_key,
                                           bool is_conditional = true);
  explicit TimeOptimalParameterizationTask(std::string name,
                                           const YAML::Node& config,
                                           const TaskComposerPluginFactory& /*plugin_factory*/);
  ~TimeOptimalParameterizationTask() override = default;
  TimeOptimalParameterizationTask(const TimeOptimalParameterizationTask&) = delete;
  TimeOptimalParameterizationTask& operator=(const TimeOptimalParameterizationTask&) = delete;
  TimeOptimalParameterizationTask(TimeOptimalParameterizationTask&&) = delete;
  TimeOptimalParameterizationTask& operator=(TimeOptimalParameterizationTask&&) = delete;

  bool operator==(const TimeOptimalParameterizationTask& rhs) const;
  bool operator!=(const TimeOptimalParameterizationTask& rhs) const;

  /**
   * @brief Unflattens a composite from TOTG back into the format of the input pattern
   *
   * Note that this algorithm will tend to shift points slightly forward because of the tolerance. e.g., if the curve
   * that represents a waypoint is actually at index 40 but index 38 is where the tolerance is first satisfied, index 39
   * and 40 will be placed in the next subcomposite
   * @param flattened_input Composite from TOTG that is completely flat
   * @param pattern Composite that was the input to TOTG. flattened_input is compared to this to find the corresponding
   * resampled points
   * @param tolerance Tolerance passed to TOTG. Thi is used to determine which points correspond
   * @return The unflattened Composite. It should have the same subcomposite structure as pattern.
   */
  static CompositeInstruction unflatten(const CompositeInstruction& flattened_input,
                                        const CompositeInstruction& pattern,
                                        double tolerance);

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TimeOptimalParameterizationTask, "TimeOptimalParameterizationTask")
#endif  // TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H

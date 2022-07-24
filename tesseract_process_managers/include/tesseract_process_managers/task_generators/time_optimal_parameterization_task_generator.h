/**
 * @file time_optimal_parameterization_task_generator.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
class TimeOptimalParameterizationTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<TimeOptimalParameterizationTaskGenerator>;

  TimeOptimalParameterizationTaskGenerator(
      std::string name = profile_ns::TIME_OPTIMAL_PARAMETERIZATION_DEFAULT_NAMESPACE);

  ~TimeOptimalParameterizationTaskGenerator() override = default;
  TimeOptimalParameterizationTaskGenerator(const TimeOptimalParameterizationTaskGenerator&) = delete;
  TimeOptimalParameterizationTaskGenerator& operator=(const TimeOptimalParameterizationTaskGenerator&) = delete;
  TimeOptimalParameterizationTaskGenerator(TimeOptimalParameterizationTaskGenerator&&) = delete;
  TimeOptimalParameterizationTaskGenerator& operator=(TimeOptimalParameterizationTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;

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
};

class TimeOptimalTrajectoryGenerationTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<TimeOptimalTrajectoryGenerationTaskInfo>;
  using ConstPtr = std::shared_ptr<const TimeOptimalTrajectoryGenerationTaskInfo>;

  TimeOptimalTrajectoryGenerationTaskInfo() = default;
  TimeOptimalTrajectoryGenerationTaskInfo(
      std::size_t unique_id,
      std::string name = profile_ns::TIME_OPTIMAL_PARAMETERIZATION_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

  bool operator==(const TimeOptimalTrajectoryGenerationTaskInfo& rhs) const;
  bool operator!=(const TimeOptimalTrajectoryGenerationTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TimeOptimalTrajectoryGenerationTaskInfo,
                        "TimeOptimalTrajectoryGenerationTaskInfo")
#endif  // TESSERACT_PROCESS_MANAGERS_ITERATIVE_SPLINE_PARAMETERIZATION_TASK_GENERATOR_H

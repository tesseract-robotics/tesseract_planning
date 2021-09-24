/**
 * @file time_optimal_trajectory_generation_task_generator.h
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

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::TimeOptimalTrajectoryGenerationProfile)
%ignore TimeOptimalTrajectoryGenerationTaskGenerator;
%ignore TimeOptimalTrajectoryGenerationTaskInfo;
#endif  // SWIG

namespace tesseract_planning
{
struct TimeOptimalTrajectoryGenerationProfile
{
  using Ptr = std::shared_ptr<TimeOptimalTrajectoryGenerationProfile>;
  using ConstPtr = std::shared_ptr<const TimeOptimalTrajectoryGenerationProfile>;

  TimeOptimalTrajectoryGenerationProfile(double max_velocity_scaling_factor = 1.0,
                                         double max_acceleration_scaling_factor = 1.0,
                                         double path_tolerance = 0.1,
                                         double resample_dt = 0.1,
                                         double min_angle_change = 0.001);

  /** @brief The max velocity scaling factor passed to the solver. Default: 1.0*/
  double max_velocity_scaling_factor;

  /** @brief The max acceleration scaling factor passed to the solver. Default: 1.0 */
  double max_acceleration_scaling_factor;

  /** @brief path_tolerance. Default: 0.1*/
  double path_tolerance;

  /** @brief Timestep used to resample trajectory. Default: 0.1*/
  double resample_dt;

  /** @brief At least one joint must change by greater than this amount for the point to be added. Default: 0.001*/
  double min_angle_change;

  /** @brief TOTG flattens the Composite structure. If this is true, the task will attempt to restructure the resulting
   * composite back into the format of the seed. This is required if MoveProfiles are specified*/
  bool unflatten{ true };

  /** @brief Tolerance used to unflatten results. When resample_dt is small, this can be close to path_tolerance. When
   * resample_dt is large, this may have to be significanly larger*/
  double unflatten_tolerance{ 0.3 };
};
using TimeOptimalTrajectoryGenerationProfileMap =
    std::unordered_map<std::string, TimeOptimalTrajectoryGenerationProfile::ConstPtr>;

class TimeOptimalTrajectoryGenerationTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<TimeOptimalTrajectoryGenerationTaskGenerator>;

  TimeOptimalTrajectoryGenerationTaskGenerator(std::string name = "TOTG");

  ~TimeOptimalTrajectoryGenerationTaskGenerator() override = default;
  TimeOptimalTrajectoryGenerationTaskGenerator(const TimeOptimalTrajectoryGenerationTaskGenerator&) = delete;
  TimeOptimalTrajectoryGenerationTaskGenerator& operator=(const TimeOptimalTrajectoryGenerationTaskGenerator&) = delete;
  TimeOptimalTrajectoryGenerationTaskGenerator(TimeOptimalTrajectoryGenerationTaskGenerator&&) = delete;
  TimeOptimalTrajectoryGenerationTaskGenerator& operator=(TimeOptimalTrajectoryGenerationTaskGenerator&&) = delete;

  TimeOptimalTrajectoryGenerationProfileMap composite_profiles;
  TimeOptimalTrajectoryGenerationProfileMap move_profiles;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override;

  void process(TaskInput input, std::size_t unique_id) const override;

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

  TimeOptimalTrajectoryGenerationTaskInfo(std::size_t unique_id, std::string name = "TOTG");
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_profile_type(TimeOptimalTrajectoryGenerationProfile);
#endif

#endif  // TESSERACT_PROCESS_MANAGERS_ITERATIVE_SPLINE_PARAMETERIZATION_TASK_GENERATOR_H

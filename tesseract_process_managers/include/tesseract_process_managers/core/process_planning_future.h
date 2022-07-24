/**
 * @file process_planning_future.h
 * @brief A process planning future
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_interface.h>
#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_process_managers/core/process_planning_problem.h>

namespace tesseract_planning
{
/**
 * @brief This contains the result for the process planning request
 * @details Must check the status before access the problem results to know if available.
 * @note This stores a shared future and is copy-able to allow access from multiple threads
 * @note This must not go out of scope until the process has finished
 */
struct ProcessPlanningFuture
{
  ProcessPlanningFuture();

  /** @brief This is the future return from taskflow executor.run, used to check if process has finished */
  std::shared_future<void> process_future;

  /** @brief This is used to abort the associated process and check if the process was successful */
  TaskflowInterface::Ptr interface;

  /** @brief This contains the problem for the process plan. Do not access until problem results until the future state
   * is ready. */
  ProcessPlanningProblem::Ptr problem;

  /** @brief Clear all content */
  void clear();

  /** @brief Checks if the future has a shared state */
  bool valid() const;

  /**
   * @brief This checks if the process has finished
   * @return True if the process finished, otherwise false
   */
  bool ready() const;

  /** @brief Wait until the process has finished */
  void wait() const;

  /**
   * @brief Check if a process has finished for a given duration
   * @return The future status
   */
  std::future_status waitFor(const std::chrono::duration<double>& duration) const;

  /**
   * @brief Check if a process has finished up to a given time point
   * @return The future status
   */
  std::future_status waitUntil(const std::chrono::time_point<std::chrono::high_resolution_clock>& abs) const;

  bool operator==(const ProcessPlanningFuture& rhs) const;
  bool operator!=(const ProcessPlanningFuture& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ProcessPlanningFuture, "ProcessPlanningFuture")
#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_FUTURE_H

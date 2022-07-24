/**
 * @file process_planning_results.h
 * @brief A process planning results
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_RESULTS_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_RESULTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_process_managers/core/taskflow_container.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/poly/instruction_poly.h>

namespace tesseract_planning
{
struct ProcessPlanningProblem
{
  using Ptr = std::shared_ptr<ProcessPlanningProblem>;
  using ConstPtr = std::shared_ptr<const ProcessPlanningProblem>;
  using UPtr = std::unique_ptr<ProcessPlanningProblem>;
  using ConstUPtr = std::unique_ptr<const ProcessPlanningProblem>;

  ProcessPlanningProblem() = default;
  ~ProcessPlanningProblem() = default;
  ProcessPlanningProblem(const ProcessPlanningProblem& other);
  ProcessPlanningProblem& operator=(const ProcessPlanningProblem& other);
  ProcessPlanningProblem(ProcessPlanningProblem&&) = delete;
  ProcessPlanningProblem& operator=(ProcessPlanningProblem&&) = delete;

  /** @brief The name of the Process Pipeline (aka. Taskflow) to use */
  std::string name;

  /** @brief The problem's environment */
  tesseract_environment::Environment::ConstPtr env;

  /** @brief The stored input to the process */
  std::unique_ptr<InstructionPoly> input{ nullptr };

  /** @brief The results to the process */
  std::unique_ptr<InstructionPoly> results{ nullptr };

  /** @brief The stored global manipulator info */
  std::unique_ptr<const tesseract_common::ManipulatorInfo> global_manip_info{ nullptr };

  /** @brief The stored plan profile remapping */
  std::unique_ptr<const PlannerProfileRemapping> plan_profile_remapping{ nullptr };

  /** @brief The stored composite profile remapping */
  std::unique_ptr<const PlannerProfileRemapping> composite_profile_remapping{ nullptr };

  /**
   * @brief The taskflow container returned from the TaskflowGenerator that must remain during taskflow execution
   * @details This is filled out by the planning server and user should not use this directly
   */
  TaskflowContainer taskflow_container;

  bool operator==(const ProcessPlanningProblem& rhs) const;
  bool operator!=(const ProcessPlanningProblem& rhs) const;

private:
  void copy(const ProcessPlanningProblem& other);

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::ProcessPlanningProblem, "ProcessPlanningProblem")
BOOST_CLASS_VERSION(tesseract_planning::ProcessPlanningProblem, 1)  // Adding name

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_RESULTS_H

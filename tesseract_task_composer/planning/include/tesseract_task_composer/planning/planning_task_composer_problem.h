/**
 * @file planning task_composer_problem.h
 * @brief A task composer server planning problem
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
#ifndef TESSERACT_TASK_COMPOSER_PLANNING_TASK_COMPOSER_PROBLEM_H
#define TESSERACT_TASK_COMPOSER_PLANNING_TASK_COMPOSER_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

namespace tesseract_planning
{
struct PlanningTaskComposerProblem : public TaskComposerProblem
{
  using Ptr = std::shared_ptr<PlanningTaskComposerProblem>;
  using ConstPtr = std::shared_ptr<const PlanningTaskComposerProblem>;
  using UPtr = std::unique_ptr<PlanningTaskComposerProblem>;
  using ConstUPtr = std::unique_ptr<const PlanningTaskComposerProblem>;

  PlanningTaskComposerProblem(std::string name = "unset");
  PlanningTaskComposerProblem(ProfileDictionary::ConstPtr profiles, std::string name = "unset");

  PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                              tesseract_common::ManipulatorInfo manip_info,
                              ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset");

  PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                              tesseract_common::ManipulatorInfo manip_info,
                              ProfileRemapping move_profile_remapping,
                              ProfileRemapping composite_profile_remapping,
                              ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset");

  PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                              ProfileRemapping move_profile_remapping,
                              ProfileRemapping composite_profile_remapping,
                              ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset");

  PlanningTaskComposerProblem(tesseract_environment::Environment::ConstPtr env,
                              ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset");

  PlanningTaskComposerProblem(const PlanningTaskComposerProblem&) = default;
  PlanningTaskComposerProblem& operator=(const PlanningTaskComposerProblem&) = default;
  PlanningTaskComposerProblem(PlanningTaskComposerProblem&&) = default;
  PlanningTaskComposerProblem& operator=(PlanningTaskComposerProblem&&) = default;
  ~PlanningTaskComposerProblem() override = default;

  /** @brief Tesseract associated with current state of the system */
  tesseract_environment::Environment::ConstPtr env;

  /** @brief Global Manipulator Information */
  tesseract_common::ManipulatorInfo manip_info;

  /** @brief The Profiles to use */
  ProfileDictionary::ConstPtr profiles;

  /**
   * @brief This allows the remapping of the Move Profile identified in the command language to a specific profile for a
   * given motion planner.
   */
  ProfileRemapping move_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner.
   */
  ProfileRemapping composite_profile_remapping;

  TaskComposerProblem::UPtr clone() const override;

  bool operator==(const PlanningTaskComposerProblem& rhs) const;
  bool operator!=(const PlanningTaskComposerProblem& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::PlanningTaskComposerProblem, "PlanningTaskComposerProblem")

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_TASK_COMPOSER_PROBLEM_H

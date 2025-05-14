/**
 * @file kinematic_limits_check_task.h
 * @brief Kinematic limits check task
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_PLANNING_NODES_KINEMATIC_LIMITS_CHECK_TASK_H
#define TESSERACT_TASK_COMPOSER_PLANNING_NODES_KINEMATIC_LIMITS_CHECK_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class KinematicLimitsCheckTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  using Ptr = std::shared_ptr<KinematicLimitsCheckTask>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckTask>;
  using UPtr = std::unique_ptr<KinematicLimitsCheckTask>;
  using ConstUPtr = std::unique_ptr<const KinematicLimitsCheckTask>;

  KinematicLimitsCheckTask();

  explicit KinematicLimitsCheckTask(std::string name,
                                    std::string input_program_key,
                                    std::string input_environment_key,
                                    std::string input_profiles_key,
                                    bool is_conditional = true);

  explicit KinematicLimitsCheckTask(std::string name,
                                    const YAML::Node& config,
                                    const TaskComposerPluginFactory& plugin_factory);

  ~KinematicLimitsCheckTask() override = default;
  KinematicLimitsCheckTask(const KinematicLimitsCheckTask&) = delete;
  KinematicLimitsCheckTask& operator=(const KinematicLimitsCheckTask&) = delete;
  KinematicLimitsCheckTask(KinematicLimitsCheckTask&&) = delete;
  KinematicLimitsCheckTask& operator=(KinematicLimitsCheckTask&&) = delete;

  bool operator==(const KinematicLimitsCheckTask& rhs) const;
  bool operator!=(const KinematicLimitsCheckTask& rhs) const;

protected:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor /*executor*/) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::KinematicLimitsCheckTask)

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_NODES_KINEMATIC_LIMITS_CHECK_TASK_H

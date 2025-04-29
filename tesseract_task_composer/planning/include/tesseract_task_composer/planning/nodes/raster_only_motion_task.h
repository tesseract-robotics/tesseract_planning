/**
 * @file raster_only_motion_task.h
 * @brief Plans raster paths only
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <functional>
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
/**
 * @brief The RasterCtOnlyMotionTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 * }
 */
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT RasterOnlyMotionTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;

  struct TaskFactoryResults
  {
    TaskComposerNode::UPtr node;
    std::string input_key;
    std::string output_key;
  };
  using TaskFactory = std::function<TaskFactoryResults(const std::string& name, std::size_t index)>;

  RasterOnlyMotionTask();
  explicit RasterOnlyMotionTask(std::string name,
                                std::string input_program_key,
                                std::string input_environment_key,
                                std::string output_program_key,
                                bool conditional,
                                TaskFactory raster_task_factory,
                                TaskFactory transition_task_factory);

  explicit RasterOnlyMotionTask(std::string name,
                                const YAML::Node& config,
                                const TaskComposerPluginFactory& plugin_factory);

  ~RasterOnlyMotionTask() override = default;
  RasterOnlyMotionTask(const RasterOnlyMotionTask&) = delete;
  RasterOnlyMotionTask& operator=(const RasterOnlyMotionTask&) = delete;
  RasterOnlyMotionTask(RasterOnlyMotionTask&&) = delete;
  RasterOnlyMotionTask& operator=(RasterOnlyMotionTask&&) = delete;

  bool operator==(const RasterOnlyMotionTask& rhs) const;
  bool operator!=(const RasterOnlyMotionTask& rhs) const;

private:
  TaskFactory raster_task_factory_;
  TaskFactory transition_task_factory_;

  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor) const override final;

  static void checkTaskInput(const tesseract_common::AnyPoly& input);

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::RasterOnlyMotionTask)

#endif  // TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H

/**
 * @file raster_ft_only_global_motion_task.h
 * @brief Plans raster paths with freespace transitions
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_GLOBAL_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_GLOBAL_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
/**
 * @brief The RasterFtOnlyGlobalPipelineTask class
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
class RasterFtOnlyGlobalPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterFtOnlyGlobalPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterFtOnlyGlobalPipelineTask>;
  using UPtr = std::unique_ptr<RasterFtOnlyGlobalPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterFtOnlyGlobalPipelineTask>;

  RasterFtOnlyGlobalPipelineTask() = default;  // Required for serialization
  RasterFtOnlyGlobalPipelineTask(std::string input_key,
                                 std::string output_key,
                                 std::string name = "RasterFtOnlyGlobalPipelineTask");
  ~RasterFtOnlyGlobalPipelineTask() override = default;
  RasterFtOnlyGlobalPipelineTask(const RasterFtOnlyGlobalPipelineTask&) = delete;
  RasterFtOnlyGlobalPipelineTask& operator=(const RasterFtOnlyGlobalPipelineTask&) = delete;
  RasterFtOnlyGlobalPipelineTask(RasterFtOnlyGlobalPipelineTask&&) = delete;
  RasterFtOnlyGlobalPipelineTask& operator=(RasterFtOnlyGlobalPipelineTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterFtOnlyGlobalPipelineTask& rhs) const;
  bool operator!=(const RasterFtOnlyGlobalPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  static void checkTaskInput(const tesseract_common::Any& input);
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterFtOnlyGlobalPipelineTask, "RasterFtOnlyGlobalPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_GLOBAL_PIPELINE_TASK_H

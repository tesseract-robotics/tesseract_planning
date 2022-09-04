/**
 * @file raster_ct_global_pipeline_task.h
 * @brief Plans raster paths with cartesian transitions
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_CT_GLOBAL_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_CT_GLOBAL_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
/**
 * @brief The RasterCtGlobalPipelineTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - from start
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - to end
 * }
 */
class RasterCtGlobalPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterCtGlobalPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterCtGlobalPipelineTask>;
  using UPtr = std::unique_ptr<RasterCtGlobalPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterCtGlobalPipelineTask>;

  RasterCtGlobalPipelineTask() = default;  // Required for serialization
  RasterCtGlobalPipelineTask(std::string input_key,
                             std::string output_key,
                             std::string name = "RasterCtGlobalPipelineTask");
  ~RasterCtGlobalPipelineTask() override = default;
  RasterCtGlobalPipelineTask(const RasterCtGlobalPipelineTask&) = delete;
  RasterCtGlobalPipelineTask& operator=(const RasterCtGlobalPipelineTask&) = delete;
  RasterCtGlobalPipelineTask(RasterCtGlobalPipelineTask&&) = delete;
  RasterCtGlobalPipelineTask& operator=(RasterCtGlobalPipelineTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterCtGlobalPipelineTask& rhs) const;
  bool operator!=(const RasterCtGlobalPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtGlobalPipelineTask, "RasterCtGlobalPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_GLOBAL_PIPELINE_TASK_H

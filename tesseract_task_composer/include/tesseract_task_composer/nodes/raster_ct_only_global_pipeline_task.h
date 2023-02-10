/**
 * @file raster_ct_only_global_motion_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_GLOBAL_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_GLOBAL_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_node_names.h>

#include <tesseract_task_composer/nodes/raster_global_pipeline_task.hpp>
#include <tesseract_task_composer/nodes/simple_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_only_motion_task.h>
#include <tesseract_task_composer/nodes/descartes_global_motion_pipeline_task.h>

namespace tesseract_planning
{
/**
 * @brief The RasterCtOnlyGlobalPipelineTask class
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
using RasterCtOnlyGlobalPipelineTaskBase =
    RasterGlobalPipelineTask<SimpleMotionPipelineTask, DescartesGlobalMotionPipelineTask, RasterCtOnlyMotionTask>;
class RasterCtOnlyGlobalPipelineTask : public RasterCtOnlyGlobalPipelineTaskBase
{
public:
  using Ptr = std::shared_ptr<RasterCtOnlyGlobalPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterCtOnlyGlobalPipelineTask>;
  using UPtr = std::unique_ptr<RasterCtOnlyGlobalPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterCtOnlyGlobalPipelineTask>;

  RasterCtOnlyGlobalPipelineTask() = default;  // Required for serialization
  RasterCtOnlyGlobalPipelineTask(std::string input_key,
                                 std::string output_key,
                                 std::string name = node_names::RASTER_CT_O_G_PIPELINE_NAME);
  ~RasterCtOnlyGlobalPipelineTask() override = default;
  RasterCtOnlyGlobalPipelineTask(const RasterCtOnlyGlobalPipelineTask&) = delete;
  RasterCtOnlyGlobalPipelineTask& operator=(const RasterCtOnlyGlobalPipelineTask&) = delete;
  RasterCtOnlyGlobalPipelineTask(RasterCtOnlyGlobalPipelineTask&&) = delete;
  RasterCtOnlyGlobalPipelineTask& operator=(RasterCtOnlyGlobalPipelineTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtOnlyGlobalPipelineTaskBase, "RasterCtOnlyGlobalPipelineTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtOnlyGlobalPipelineTask, "RasterCtOnlyGlobalPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_GLOBAL_PIPELINE_TASK_H

/**
 * @file raster_ct_pipeline_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_CT_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_CT_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
/**
 * @brief The RasterCtPipelineTask class
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
class RasterCtPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterCtPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterCtPipelineTask>;
  using UPtr = std::unique_ptr<RasterCtPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterCtPipelineTask>;

  RasterCtPipelineTask() = default;  // Required for serialization
  RasterCtPipelineTask(std::string input_key, std::string output_key, std::string name = "RasterCtPipelineTask");
  ~RasterCtPipelineTask() override = default;
  RasterCtPipelineTask(const RasterCtPipelineTask&) = delete;
  RasterCtPipelineTask& operator=(const RasterCtPipelineTask&) = delete;
  RasterCtPipelineTask(RasterCtPipelineTask&&) = delete;
  RasterCtPipelineTask& operator=(RasterCtPipelineTask&&) = delete;

  bool operator==(const RasterCtPipelineTask& rhs) const;
  bool operator!=(const RasterCtPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtPipelineTask, "RasterCtPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_PIPELINE_TASK_H

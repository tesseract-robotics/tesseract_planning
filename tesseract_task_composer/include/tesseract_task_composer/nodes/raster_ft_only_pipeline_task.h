/**
 * @file raster_ft_only_motion_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_node_names.h>

namespace tesseract_planning
{
/**
 * @brief The RasterFtOnlyPipelineTask class
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
class RasterFtOnlyPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterFtOnlyPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterFtOnlyPipelineTask>;
  using UPtr = std::unique_ptr<RasterFtOnlyPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterFtOnlyPipelineTask>;

  RasterFtOnlyPipelineTask() = default;  // Required for serialization
  RasterFtOnlyPipelineTask(std::string input_key,
                           std::string output_key,
                           std::string name = node_names::RASTER_FT_O_PIPELINE_NAME);
  ~RasterFtOnlyPipelineTask() override = default;
  RasterFtOnlyPipelineTask(const RasterFtOnlyPipelineTask&) = delete;
  RasterFtOnlyPipelineTask& operator=(const RasterFtOnlyPipelineTask&) = delete;
  RasterFtOnlyPipelineTask(RasterFtOnlyPipelineTask&&) = delete;
  RasterFtOnlyPipelineTask& operator=(RasterFtOnlyPipelineTask&&) = delete;

  bool operator==(const RasterFtOnlyPipelineTask& rhs) const;
  bool operator!=(const RasterFtOnlyPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterFtOnlyPipelineTask, "RasterFtOnlyPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_FT_ONLY_PIPELINE_TASK_H

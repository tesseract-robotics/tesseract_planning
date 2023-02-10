/**
 * @file raster_ft_motion_task.h
 * @brief Raster motion task with freespace transitions
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_names.h>
#include <tesseract_common/any_poly.h>

#include <tesseract_task_composer/nodes/raster_motion_task.hpp>
#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/freespace_motion_pipeline_task.h>

namespace tesseract_planning
{
using RasterFtMotionTaskBase =
    RasterMotionTask<FreespaceMotionPipelineTask, CartesianMotionPipelineTask, FreespaceMotionPipelineTask>;
class RasterFtMotionTask : public RasterFtMotionTaskBase
{
public:
  using Ptr = std::shared_ptr<RasterFtMotionTask>;
  using ConstPtr = std::shared_ptr<const RasterFtMotionTask>;
  using UPtr = std::unique_ptr<RasterFtMotionTask>;
  using ConstUPtr = std::unique_ptr<const RasterFtMotionTask>;

  RasterFtMotionTask() = default;  // Required for serialization
  RasterFtMotionTask(std::string input_key,
                     std::string output_key,
                     bool is_conditional = true,
                     std::string name = node_names::RASTER_FT_MOTION_TASK_NAME);
  ~RasterFtMotionTask() override = default;
  RasterFtMotionTask(const RasterFtMotionTask&) = delete;
  RasterFtMotionTask& operator=(const RasterFtMotionTask&) = delete;
  RasterFtMotionTask(RasterFtMotionTask&&) = delete;
  RasterFtMotionTask& operator=(RasterFtMotionTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterFtMotionTaskBase, "RasterFtMotionTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterFtMotionTask, "RasterFtMotionTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H
